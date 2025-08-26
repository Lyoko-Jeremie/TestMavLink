import {
    MavLinkData,
    MavLinkPacket,
    MavLinkPacketParser,
    MavLinkPacketRegistry,
    MavLinkPacketSplitter,
    MavLinkProtocol,
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

export interface PackAndDataType<Data extends MavLinkData = MavLinkData> {
    packet: MavLinkPacket;
    data: Data;
}

export class MavLinkPacket2Data {
    constructor(
        public REGISTRY: MavLinkPacketRegistry,
        private mavLinkDecodeStream: PassThrough,
        private mavLinkPacketObservable: Observable<MavLinkPacket>,
    ) {
    }

    static create(
        registry: MavLinkPacketRegistry,
        mavLinkDecodeStream: PassThrough,
        mavLinkPacketObservable: Observable<MavLinkPacket>,
    ): MavLinkPacket2Data {
        return new MavLinkPacket2Data(
            registry,
            mavLinkDecodeStream,
            mavLinkPacketObservable,
        );
    }

    public process(packet: MavLinkPacket): MavLinkData | undefined {
        // 使用注册表将消息ID映射到数据类，并使用协议解析器将负载数据转换为相应的数据对象
        const clazz = this.REGISTRY[packet.header.msgid];
        if (clazz) {
            const data = packet.protocol.data(packet.payload, clazz);
            return data;
        } else {
            console.warn('[MavLinkPacket2Data] Cannot decode packet :', packet.header.msgid, packet);
        }
        return undefined;
    }

    public stream(): MavLinkPacket2DataTransform {
        return this.mavLinkDecodeStream.pipe(new MavLinkPacket2DataTransform(this.process.bind(this)));
    }

    protected subjectPackAndData?: Observable<PackAndDataType>;

    public observable(): Observable<MavLinkData> {
        return this.observablePackAndData().pipe(
            map((d: PackAndDataType) => {
                return d.data;
            }),
        );
    }

    public observablePackAndData(): Observable<PackAndDataType> {
        if (!this.subjectPackAndData) {
            this.subjectPackAndData = this.mavLinkPacketObservable.pipe(
                map((packet: MavLinkPacket) => {
                    return {
                        packet: packet,
                        data: this.process(packet),
                    };
                }),
                filter((T): T is PackAndDataType => T.data !== undefined),
            );
        }
        return this.subjectPackAndData;
    }
}

export class MavLinkDecodeStream {
    private mavLinkDecodeStream: PassThrough = new PassThrough();

    private outputStream: MavLinkPacketParser;

    private mavLinkPacketObservable: Subject<MavLinkPacket> = new Subject<MavLinkPacket>();

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
            this.mavLinkPacketObservable.next(data);
        });
    }

    public get readPacketStream(): PassThrough {
        return this.outputStream;
    }

    public get observablePacket(): Subject<MavLinkPacket> {
        return this.mavLinkPacketObservable;
    }

    public get readDataStream(): PassThrough {
        return MavLinkPacket2Data.create(
            this.REGISTRY,
            this.mavLinkDecodeStream,
            this.mavLinkPacketObservable,
        ).stream();
    }

    public get observableData(): Observable<MavLinkData> {
        return MavLinkPacket2Data.create(
            this.REGISTRY,
            this.mavLinkDecodeStream,
            this.mavLinkPacketObservable,
        ).observable();
    }

    public get observablePackAndData(): Observable<PackAndDataType> {
        return MavLinkPacket2Data.create(
            this.REGISTRY,
            this.mavLinkDecodeStream,
            this.mavLinkPacketObservable,
        ).observablePackAndData();
    }

}

export class CustomProtocolTransformManager {

    // 序列号从1开始
    private seq = 1;
    private customProtocolReadStream: PassThrough;
    private customProtocolWriteStream: PassThrough;

    private mavLinkDecodeStreamTable: Map<number, MavLinkDecodeStream> = new Map<number, MavLinkDecodeStream>();

    mavLinkAllDataSubject: Subject<{ id: number, data: MavLinkData }> = new Subject();
    mavLinkAllPackAndDataSubject: Subject<{ id: number, packAndData: PackAndDataType }> = new Subject();

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
        port.pipe(new CustomProtocolTransformFromSerialPort(true)).pipe(this.customProtocolReadStream);

        this.customProtocolReadStream.on('data', (data: CustomProtocolPackage) => {
            this.onPacket(data);
        });

        // to SerialPort
        // 创建一个 PassThrough 流，用于向 SerialPort 写入数据，并加入自定义协议封装器
        this.customProtocolWriteStream = new PassThrough({
            readableObjectMode: true,
            writableObjectMode: true,
        });
        this.customProtocolWriteStream.pipe(new CustomProtocolTransformToSerialPort(true)).pipe(port);

    }

    private onPacket(pack: CustomProtocolPackage): void {
        this.debug && console.log('[CPT] Received a pack:', pack);

        // 检查是否有对应的 mavLinkDecodeStream
        let mavLinkDecodeStream = this.mavLinkDecodeStreamTable.get(pack.id);
        if (!mavLinkDecodeStream) {
            mavLinkDecodeStream = new MavLinkDecodeStream(this.REGISTRY);
            this.mavLinkDecodeStreamTable.set(pack.id, mavLinkDecodeStream);
            mavLinkDecodeStream.observableData.pipe(map(T => {
                return {id: pack.id, data: T};
            })).subscribe(this.mavLinkAllDataSubject);
            mavLinkDecodeStream.observablePackAndData.pipe(map(T => {
                return {id: pack.id, packAndData: T};
            })).subscribe(this.mavLinkAllPackAndDataSubject);
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
            mavLinkDecodeStream.observableData.pipe(map(T => {
                return {id: deviceId, data: T};
            })).subscribe(this.mavLinkAllDataSubject);
            mavLinkDecodeStream.observablePackAndData.pipe(map(T => {
                return {id: deviceId, packAndData: T};
            })).subscribe(this.mavLinkAllPackAndDataSubject);
        }

        return mavLinkDecodeStream;
    }

    getMavLinkAllDataObservable() {
        return this.mavLinkAllDataSubject.asObservable();
    }

    getMavLinkAllPackAndDataObservable() {
        return this.mavLinkAllPackAndDataSubject;
    }

    public debugSerializeMavLinkMsg(msg: MavLinkData): Buffer {
        return this.protocol.serialize(msg, this.seq + 1);
    }

    /**
     * 发送 mavlink 消息
     * @param msg - mavlink 消息数据
     * @param targetId - 目标设备 ID，0xFF 表示广播
     * @returns 是否发送成功
     */
    public async sendMsg(msg: MavLinkData, targetId: number): Promise<number | undefined> {
        if ((targetId & 0xFF) !== targetId) {
            console.error('[CPT] sendMsg invalid targetId :', targetId);
            return undefined;
        }

        const seq = this.seq++;
        const packBuf = this.protocol.serialize(msg, seq);

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
        }).then(() => seq).catch(e => {
            console.error('[CPT] sendMsg error :', e);
            return undefined;
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

