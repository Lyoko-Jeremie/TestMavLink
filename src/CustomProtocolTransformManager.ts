import {
    MavLinkData,
    MavLinkPacket,
    MavLinkPacketParser,
    MavLinkPacketRegistry,
    MavLinkPacketSplitter,
    MavLinkProtocol
} from "node-mavlink";
import {Duplex, PassThrough, Transform, TransformCallback, Stream, Writable} from "stream";
import {
    CustomProtocolPackage,
    CustomProtocolTransformFromSerialPort, CustomProtocolTransformToSerialPort,
} from "./CustomProtocolTransform";
import {filter, map, Observable, Subject} from 'rxjs';

export class MavLinkPacket2DataTransform extends Transform {
    constructor(
        public process: (packet: MavLinkPacket) => MavLinkData | undefined,
        options?: any,
    ) {
        super(options);
    }

    _transform(packet: MavLinkPacket, encoding: BufferEncoding, callback: TransformCallback): void {
        const data = this.process(packet);
        if (!data) {
            return callback();
        }
        callback(null, data);
    }
}

export class MavLinkPacket2Data {
    constructor(
        public REGISTRY: MavLinkPacketRegistry,
    ) {
    }

    static create(
        registry: MavLinkPacketRegistry,
    ): MavLinkPacket2Data {
        return new MavLinkPacket2Data(registry);
    }

    public process(packet: MavLinkPacket): MavLinkData | undefined {
        // 使用注册表将消息ID映射到数据类，并使用协议解析器将负载数据转换为相应的数据对象
        const clazz = this.REGISTRY[packet.header.msgid];
        if (clazz) {
            const data = packet.protocol.data(packet.payload, clazz);
            return data;
        }
        return undefined;
    }

    public stream<T extends Stream>(s: T): MavLinkPacket2DataTransform {
        return s.pipe(new MavLinkPacket2DataTransform(this.process.bind(this)));
    }

    public observable(s: Observable<MavLinkPacket>): Observable<MavLinkData> {
        return s.pipe(
            map((packet: MavLinkPacket) => {
                return this.process(packet);
            }),
            filter(T => T !== undefined),
        );
    }
}

export class MavLinkDecodeStream {
    private mavLinkDecodeStream: PassThrough = new PassThrough();

    private outputStream: MavLinkPacketParser;

    private sub: Subject<MavLinkPacket> = new Subject<MavLinkPacket>();

    /**
     * 写入 serial port data 数据包到 mavLinkDecodeStream
     * @param data - mavlink 数据包的 binary 数据
     * @returns 是否成功写入
     */
    public writeData: (data: Buffer) => boolean = this.mavLinkDecodeStream.write.bind(this.mavLinkDecodeStream);

    constructor(
        public REGISTRY: MavLinkPacketRegistry,
    ) {
        this.outputStream = this.mavLinkDecodeStream
            .pipe(new MavLinkPacketSplitter())
            .pipe(new MavLinkPacketParser());

        this.outputStream.on('data', (data: MavLinkPacket) => {
            this.sub.next(data);
        });
    }

    public get readPacketStream(): PassThrough {
        return this.outputStream;
    }

    public get observablePacket(): Subject<MavLinkPacket> {
        return this.sub;
    }

    public get readDataStream(): PassThrough {
        return MavLinkPacket2Data.create(this.REGISTRY).stream(this.outputStream);
    }

    public get observableData(): Observable<MavLinkData> {
        return MavLinkPacket2Data.create(this.REGISTRY).observable(this.sub);
    }

}

export class CustomProtocolTransformManager {

    // 序列号从0开始
    private seq = 0;
    private customProtocolReadStream: PassThrough;
    private customProtocolWriteStream: PassThrough;

    private mavLinkDecodeStreamTable: Map<number, MavLinkDecodeStream> = new Map<number, MavLinkDecodeStream>();

    constructor(
        public port: Duplex,
        public protocol: MavLinkProtocol,
        public REGISTRY: MavLinkPacketRegistry,
        public debug: boolean = false,
    ) {

        // from SerialPort
        // 创建一个 PassThrough 流，用于从 SerialPort 读取数据，并加入自定义协议解析器
        this.customProtocolReadStream = new PassThrough({
            readableObjectMode: true,
            writableObjectMode: true,
        })
        port.pipe(new CustomProtocolTransformFromSerialPort(false)).pipe(this.customProtocolReadStream);

        this.customProtocolReadStream.on('data', (data: CustomProtocolPackage) => {
            this.onPacket(data);
        });

        // to SerialPort
        // 创建一个 PassThrough 流，用于向 SerialPort 写入数据，并加入自定义协议封装器
        this.customProtocolWriteStream = new PassThrough({
            readableObjectMode: true,
            writableObjectMode: true,
        });
        this.customProtocolWriteStream.pipe(new CustomProtocolTransformToSerialPort(false)).pipe(port);

    }

    private onPacket(pack: CustomProtocolPackage): void {
        this.debug && console.log('[CPT] Received a pack:', pack);

        // 检查是否有对应的 mavLinkDecodeStream
        let mavLinkDecodeStream = this.mavLinkDecodeStreamTable.get(pack.id);
        if (!mavLinkDecodeStream) {
            mavLinkDecodeStream = new MavLinkDecodeStream(this.REGISTRY);
            this.mavLinkDecodeStreamTable.set(pack.id, mavLinkDecodeStream);
        }

        // 将数据写入对应的 mavLinkDecodeStream
        mavLinkDecodeStream.writeData(Buffer.from(pack.payload));
    }

    public getDecoderStream(deviceId: number): MavLinkDecodeStream {
        // 检查是否有对应的 mavLinkDecodeStream
        let mavLinkDecodeStream = this.mavLinkDecodeStreamTable.get(deviceId);
        if (!mavLinkDecodeStream) {
            mavLinkDecodeStream = new MavLinkDecodeStream(this.REGISTRY);
            this.mavLinkDecodeStreamTable.set(deviceId, mavLinkDecodeStream);
        }

        return mavLinkDecodeStream;
    }

    /**
     * 发送 mavlink 消息
     * @param msg - mavlink 消息数据
     * @param targetId - 目标设备 ID，0xFF 表示广播
     * @returns 是否发送成功
     */
    public async sendMsg(msg: MavLinkData, targetId: number): Promise<boolean> {
        if (targetId && 0xFF !== targetId) {
            console.error('[CPT] sendMsg invalid targetId :', targetId);
            return false;
        }

        const packBuf = this.protocol.serialize(msg, this.seq++);

        return new Promise((resolve, reject) => {
            // writeStreamWithWait(this.customProtocolWriteStream,
            //     {
            //         id: targetId,
            //         payload: packBuf,
            //     } satisfies CustomProtocolPackage,
            //     (err: any) => {
            //         if (err) {
            //             reject(err);
            //         } else {
            //             resolve(true);
            //         }
            //     });
            this.customProtocolWriteStream.write(
                {
                    id: targetId,
                    payload: packBuf,
                } satisfies CustomProtocolPackage,
                (err: any) => {
                    if (err) {
                        reject(err);
                    } else {
                        resolve(true);
                    }
                });
        });
    }

}

// Stream.Writable.write
function writeStreamWithWait(stream: Writable, data: any, cb: (...arg: any) => void): void {
    if (!stream.write(data)) {
        stream.once('drain', cb);
    } else {
        process.nextTick(cb);
    }
}

