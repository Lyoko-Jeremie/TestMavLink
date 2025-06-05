import {SerialPort} from 'serialport';
import {
    ardupilotmega,
    common,
    MavLinkPacketParser,
    MavLinkPacketRegistry,
    MavLinkPacketSplitter,
    MavLinkProtocolV2,
    minimal,
    send,
} from 'node-mavlink';
import {PassThrough, Transform, TransformCallback} from 'stream';

console.log('Happy developing ✨');

// create a registry of mappings between a message id and a data class
const REGISTRY: MavLinkPacketRegistry = {
    ...minimal.REGISTRY,
    ...common.REGISTRY,
    ...ardupilotmega.REGISTRY,
};

class CustomProtocolTransformToSerialPort extends Transform {
    constructor(options?: any) {
        super(options);
    }

    _transform(chunk: Buffer, encoding: BufferEncoding, callback: TransformCallback): void {
        // 帧头1	帧头2	ID	                    数据长度	    PLAYLOAD(data)	    uint8_t校验和	帧尾
        // 0xAA	0xBB	1-16（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
        //
        // 备注：id为1-16个天空端的设备ID
        //       playload为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到playload
        //
        console.log('[To SP] chunk', chunk);
        console.log('[To SP] chunk', new Uint8Array(chunk));
        const data = Buffer.from([0xAA, 0xBB, 1, chunk.length, ...chunk, 0, 0xCC]); // 0xCC is the frame tail
        const checksum = data.subarray(0, -2).reduce((acc, byte) => acc + byte, 0) & 0xFF; // simple checksum
        data[data.length - 2] = checksum; // set the checksum byte
        console.log('[To SP] data', data);
        console.log('[To SP] data', new Uint8Array(data));
        callback(null, data);
    }
}

class CustomProtocolTransformFromSerialPort extends Transform {
    constructor(options?: any) {
        super(options);
    }

    // buffer the incoming data until we have a complete frame
    private buffer: Uint8Array = new Uint8Array();

    _transform(chunk: Buffer, encoding: BufferEncoding, callback: TransformCallback): void {
        // 帧头1	帧头2	ID	                    数据长度	    PLAYLOAD(data)	    uint8_t校验和	帧尾
        // 0xAA	0xBB	1-16（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
        //
        // 备注：id为1-16个天空端的设备ID
        //       playload为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到playload
        //

        // cache data
        const newBuffer = new Uint8Array(this.buffer.length + chunk.length);
        newBuffer.set(this.buffer);
        newBuffer.set(chunk, this.buffer.length);
        this.buffer = newBuffer;

        // find the start of the frame
        let startIndex = -1;
        for (let i = 0; i < this.buffer.length - 3; i++) {
            if (this.buffer[i] === 0xAA && this.buffer[i + 1] === 0xBB) {
                startIndex = i;
                break;
            }
        }
        console.log('[From SP] Start index of frame:', startIndex);
        if (startIndex === -1) {
            // no start of frame found, wait for more data
            console.log('[From SP] No start of frame found, waiting for more data');
            return callback();
        }
        // get the size of the frame
        const payloadSize = this.buffer[startIndex + 3];
        // 2 bytes header, 1 byte ID, 1 byte len , 1 byte checksum , 1 byte frame tail
        const frameSize = payloadSize + 2 + 1 + 1 + 1 + 1;
        console.log('[From SP] payload size:', payloadSize);
        console.log('[From SP] Frame size:', frameSize);
        if (this.buffer.length < startIndex + frameSize) {
            // not enough data for a complete frame, wait for more data
            console.log('[From SP] Not enough data for a complete frame, waiting for more data');
            return callback();
        }
        // get the end of the frame , check if the frame ends with 0xCC
        const endIndex = startIndex + frameSize;
        if (this.buffer[endIndex - 1] !== 0xCC) {
            // bad frame end, maybe a broken frame
            console.log('[From SP] Bad frame end:', this.buffer.subarray(startIndex, endIndex));
            console.log(Buffer.from(this.buffer.subarray(startIndex, endIndex)));
            return callback();
        }

        // extract the frame
        const frame = this.buffer.subarray(startIndex, endIndex);
        console.log('[From SP] Extracted frame:', Buffer.from(frame));

        // check checksum
        const checksum = frame[frame.length - 2];
        const calculatedChecksum = frame.subarray(0, -2).reduce((acc, byte) => acc + byte, 0) & 0xFF;
        if (checksum !== calculatedChecksum) {
            // checksum error, maybe a broken frame
            console.log('[From SP] Checksum error:', checksum, '!=', calculatedChecksum);
            return callback();
        }

        // extract the payload
        const payload = frame.subarray(4, endIndex - 2); // skip header (2 bytes), ID (1 byte), length (1 byte)
        console.log('[From SP] payload');
        console.log(payload);

        // remove the processed frame from the buffer
        const remainingLength = this.buffer.length - endIndex;
        const remainData = new Uint8Array(remainingLength);
        if (remainingLength > 0) {
            remainData.set(this.buffer.subarray(endIndex), 0);
        }
        this.buffer = remainData;
        const rData = Buffer.from(payload);
        console.log('[From SP] rData', rData);
        callback(null, rData);
    }

    _flush(callback: TransformCallback): void {
        // if there is any remaining data in the buffer, emit it as a final chunk
        if (this.buffer.length > 0) {
            console.log('[From SP] Flushing remaining data:', this.buffer);
            this.push(this.buffer);
            this.buffer = new Uint8Array();
        }
        callback();
    }
}


// substitute /dev/ttyACM0 with your serial port!
const port = new SerialPort({path: 'COM4', baudRate: 115200});

// from SerialPort
const customProtocolReadStream = new PassThrough();
port.pipe(new CustomProtocolTransformFromSerialPort()).pipe(customProtocolReadStream)

// to SerialPort
const customProtocolWriteStream = new PassThrough();
customProtocolWriteStream.pipe(new CustomProtocolTransformToSerialPort()).pipe(port);

// constructing a reader that will emit each packet separately
const reader = customProtocolReadStream
    .pipe(new MavLinkPacketSplitter())
    .pipe(new MavLinkPacketParser());

reader.on('data', packet => {
    console.log(packet)

    const clazz = REGISTRY[packet.header.msgid];
    if (clazz) {
        const data = packet.protocol.data(packet.payload, clazz);
        console.log('====== Received packet:', data);
    }
});

async function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

port.on('open', async () => {
    // the port is open - we're ready to send data
    console.log('====== commandHeartbeat');
    const commandHeartbeat = new minimal.Heartbeat();
    commandHeartbeat.systemStatus = minimal.MavState.STANDBY;
    await send(customProtocolWriteStream, commandHeartbeat, new MavLinkProtocolV2());
    console.log('====== commandSystemTime');
    const commandSystemTime = new common.SystemTime();
    commandSystemTime.timeUnixUsec = BigInt(Date.now()) * BigInt(1000);
    commandSystemTime.timeBootMs = process.uptime() * 1000;
    await send(customProtocolWriteStream, commandSystemTime, new MavLinkProtocolV2());
    console.log('====== sendEnd');
});

