import {Transform, TransformCallback} from 'stream';
import {assert} from 'tsafe';

//      协议识别码包含
//      COMMAND_MSG(0)、     // MavLink数据传输
//      SETADDR_PAIR(32)、
//      SETADDR_PAIR_ACK(64)、
//      SETADDR_PAIR_REQUEST(96)、
//      SETADDR_PAIR_INFO(128)、
export enum ProtocolMode {
    CommandMsg = 0,
    SetAddrPair = 32,
    SetAddrPairAck = 64,
    SetAddrPairRequest = 96,
    SetAddrPairInfo = 128,

    Unknown = 256,
}

export interface CustomProtocolPackage {
    payload: Uint8Array;
    deviceId: number;
    protocolCode: number;
    protocolMode: ProtocolMode;
}


export class CustomProtocolTransformToSerialPort extends Transform {
    constructor(
        public debug?: boolean,
        options?: any,
    ) {
        super(Object.assign({
            readableObjectMode: true,
            writableObjectMode: true,
        }, options));
    }

    _transform(pack: CustomProtocolPackage, encoding: BufferEncoding, callback: TransformCallback): void {
        // 封装包
        // 帧头1	帧头2	ID	                            数据长度	    payload(data)	    uint8_t校验和	帧尾
        // 0xAA	0xBB	协议识别码+0-15（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
        //
        // 备注：id为 0-15个天空端的设备ID + 协议识别码(以高位0xF0区域来表示)
        //      协议识别码包含
        //      COMMAND_MSG(0)、
        //      SETADDR_PAIR(32)、
        //      SETADDR_PAIR_ACK(64)、
        //      SETADDR_PAIR_REQUEST(96)、
        //      SETADDR_PAIR_INFO(128)、
        //
        //
        //      当协议识别码为 COMMAND_MSG 时：
        //      payload 为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到payload
        //      当协议识别码为 SETADDR_PAIR 时： payload 为 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801 原始包字节
        //      当协议识别码为 SETADDR_PAIR_ACK 时： payload 为 uint8_t ack (0:失败，1:成功)
        //      当协议识别码为 SETADDR_PAIR_REQUEST 时： payload 为 空， 0字节长。
        //      当协议识别码为 SETADDR_PAIR_INFO 时： payload 中为读取到的地址数据，结构体如下
        //
        //
        const chunk = pack.payload;
        this.debug && console.log('[To SP] chunk', chunk);
        this.debug && console.log('[To SP] chunk', new Uint8Array(chunk));
        const idField = ((pack.protocolCode & 0xF0) | (pack.deviceId & 0x0F)) & 0xFF;
        const data = Buffer.from([0xAA, 0xBB, idField, chunk.length, ...chunk, 0, 0xCC]); // 0xCC is the frame tail
        const checksum = data.subarray(0, -2).reduce((acc, byte) => acc + byte, 0) & 0xFF; // simple checksum
        data[data.length - 2] = checksum; // set the checksum byte
        this.debug && console.log('[To SP] data', data);
        this.debug && console.log('[To SP] data', new Uint8Array(data));
        callback(null, data);
    }
}

type DetectFirstPackFromUint8ArrayReturnType = {
    startIndex: number,
    endIndex: number,
    payloadSize: number,
    frameSize: number,
    checksum: number,
    calculatedChecksum: number,
    idField: number,
    payloadSlice: Uint8Array,
}

function detectFirstPackFromUint8Array(debug: boolean, buffer: Uint8Array): {
    ok?: DetectFirstPackFromUint8ArrayReturnType,
    bad?: {
        skipEndIndex: number,
    },
} | undefined {
    // 封装包
    // 帧头1	帧头2	ID	                            数据长度	    payload(data)	    uint8_t校验和	帧尾
    // 0xAA	0xBB	协议识别码+0-15（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
    //
    // 备注：id为 0-15个天空端的设备ID + 协议识别码(以高位0xF0区域来表示)
    //      协议识别码包含
    //      COMMAND_MSG(0)、
    //      SETADDR_PAIR(32)、
    //      SETADDR_PAIR_ACK(64)、
    //      SETADDR_PAIR_REQUEST(96)、
    //      SETADDR_PAIR_INFO(128)、
    //
    //
    //      当协议识别码为 COMMAND_MSG 时：
    //      payload 为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到payload
    //      当协议识别码为 SETADDR_PAIR 时： payload 为 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801 原始包字节
    //      当协议识别码为 SETADDR_PAIR_ACK 时： payload 为 uint8_t ack (0:失败，1:成功)
    //      当协议识别码为 SETADDR_PAIR_REQUEST 时： payload 为 空， 0字节长。
    //      当协议识别码为 SETADDR_PAIR_INFO 时： payload 中为读取到的地址数据，结构体如下
    //
    //


    // find the start of the frame
    let startIndex = -1;
    for (let i = 0; i < buffer.length - 3; i++) {
        if (buffer[i] === 0xAA && buffer[i + 1] === 0xBB) {
            startIndex = i;
            break;
        }
    }
    debug && console.log('[From SP] Start index of frame:', startIndex);
    if (startIndex === -1) {
        // no start of frame found, wait for more data
        console.log('[From SP] No start of frame found, waiting for more data');
        console.log(buffer);
        return undefined;
    }
    if (startIndex > 0) {
        debug && console.log('[From SP] Some data be trimmed before the frame start, warning.', buffer.subarray(0, startIndex));
    }
    // get the size of the frame
    const payloadSize = buffer[startIndex + 3];
    // 2 bytes header, 1 byte ID, 1 byte len , 1 byte checksum , 1 byte frame tail
    const frameSize = payloadSize + 2 + 1 + 1 + 1 + 1;
    debug && console.log('[From SP] payload size:', payloadSize);
    debug && console.log('[From SP] Frame size:', frameSize);
    if (buffer.length < startIndex + frameSize) {
        // not enough data for a complete frame, wait for more data
        debug && console.log('[From SP] Not enough data for a complete frame, waiting for more data');
        return undefined;
    }
    // get the end of the frame , check if the frame ends with 0xCC
    const endIndex = startIndex + frameSize;
    if (buffer[endIndex - 1] !== 0xCC) {
        // bad frame end, maybe a broken frame
        console.error('[From SP] Bad frame end:', buffer.subarray(startIndex, endIndex));
        console.error(Buffer.from(buffer.subarray(startIndex, endIndex)));
        // TODO skip to next 0xCC ??????????
        // try to find next 0xccaabb
        let nextStartIndex = -1;
        for (let i = 0; i < buffer.length - 3; i++) {
            if (buffer[i] === 0xCC && buffer[i + 1] === 0xAA && buffer[i + 2] === 0xBB) {
                nextStartIndex = i + 1;
                return {
                    bad: {
                        skipEndIndex: nextStartIndex,
                    },
                };
            }
        }
        return undefined;
    }

    // extract the frame
    const frame = buffer.subarray(startIndex, endIndex);
    debug && console.log('[From SP] Extracted frame:', Buffer.from(frame));

    // check checksum
    const checksum = frame[frame.length - 2];
    const calculatedChecksum = frame.subarray(0, -2).reduce((acc, byte) => acc + byte, 0) & 0xFF;
    if (checksum !== calculatedChecksum) {
        // checksum error, maybe a broken frame
        console.error('[From SP] Checksum error:', checksum, '!=', calculatedChecksum);
        // TODO remove this frame from the buffer
        return undefined;
    }

    const idField = frame[2]; // ID is the 3rd byte in the frame

    // extract the payload
    const payload = frame.subarray(4, endIndex - 2); // skip header (2 bytes), ID (1 byte), length (1 byte)
    debug && console.log('[From SP] payload');
    debug && console.log(payload);

    return {
        ok: {
            startIndex,
            endIndex,
            payloadSize,
            frameSize,
            checksum,
            calculatedChecksum,
            idField: idField,
            payloadSlice: payload,
        },
    };
}

export class CustomProtocolTransformFromSerialPort extends Transform {
    constructor(
        public debug?: boolean,
        options?: any,
    ) {
        super(Object.assign({
            readableObjectMode: true,
            writableObjectMode: true,
        }, options));
    }

    // buffer the incoming data until we have a complete frame
    private buffer: Uint8Array = new Uint8Array();

    _transform(chunk: Buffer, encoding: BufferEncoding, callback: TransformCallback): void {
        // 封装包
        // 帧头1	帧头2	ID	                            数据长度	    payload(data)	    uint8_t校验和	帧尾
        // 0xAA	0xBB	协议识别码+0-15（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
        //
        // 备注：id为 0-15个天空端的设备ID + 协议识别码(以高位0xF0区域来表示)
        //      协议识别码包含
        //      COMMAND_MSG(0)、
        //      SETADDR_PAIR(32)、
        //      SETADDR_PAIR_ACK(64)、
        //      SETADDR_PAIR_REQUEST(96)、
        //      SETADDR_PAIR_INFO(128)、
        //
        //
        //      当协议识别码为 COMMAND_MSG 时：
        //      payload 为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到payload
        //      当协议识别码为 SETADDR_PAIR 时： payload 为 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801 原始包字节
        //      当协议识别码为 SETADDR_PAIR_ACK 时： payload 为 uint8_t ack (0:失败，1:成功)
        //      当协议识别码为 SETADDR_PAIR_REQUEST 时： payload 为 空， 0字节长。
        //      当协议识别码为 SETADDR_PAIR_INFO 时： payload 中为读取到的地址数据，结构体如下
        //
        //
        this.debug && console.log('[From SP] Received chunk:', chunk);

        // cache data
        const newBuffer = new Uint8Array(this.buffer.length + chunk.length);
        newBuffer.set(this.buffer);
        newBuffer.set(chunk, this.buffer.length);
        this.buffer = newBuffer;

        const r = detectFirstPackFromUint8Array(this.debug ?? false, this.buffer);
        if (!r) {
            // no complete frame found, wait for more data
            this.debug && console.log('[From SP] No complete frame found, waiting for more data');
            return callback();
        }
        const detectedFrame = r.ok;
        if (r.bad) {
            // bad frame, skip the bad data
            this.buffer = this.buffer.subarray(r.bad!.skipEndIndex);
            return callback();
        }
        if (!detectedFrame) {
            // never go there
            console.error('CustomProtocolTransformFromSerialPort _transform() detectedFrame is undefined. never go there.');
            return callback();
        }

        // 低4位为设备id号
        const deviceId = detectedFrame.idField & 0x0F;
        // 高4位为协议识别码
        const protocolCode = detectedFrame.idField & 0xF0;
        const protocolMode = Object.values(ProtocolMode).includes(protocolCode) ? protocolCode as ProtocolMode : ProtocolMode.Unknown;

        const rData: CustomProtocolPackage = {
            payload: new Uint8Array(detectedFrame.payloadSlice.length),
            deviceId: deviceId,
            protocolCode: protocolCode,
            protocolMode: protocolMode,
        };
        rData.payload.set(detectedFrame.payloadSlice, 0);
        this.debug && console.log('[From SP] rData', rData);

        // remove the processed frame from the buffer
        const remainingLength = this.buffer.length - detectedFrame.endIndex;
        const remainData = new Uint8Array(remainingLength);
        if (remainingLength > 0) {
            remainData.set(this.buffer.subarray(detectedFrame.endIndex), 0);
        }
        this.buffer = remainData;

        callback(null, rData);
    }

    _flush(callback: TransformCallback): void {
        // if there is any remaining data in the buffer, emit it as a final chunk
        if (this.buffer.length > 0) {
            this.debug && console.log('[From SP] Flushing remaining data:', this.buffer);

            let remainingBuffer = this.buffer.subarray(0, this.buffer.length - 1);
            assert(remainingBuffer.length === this.buffer.length, ' assert (remainingBuffer.length === this.buffer.length');

            while (remainingBuffer.length > 0) {

                const r = detectFirstPackFromUint8Array(this.debug ?? false, remainingBuffer);
                if (!r) {
                    // no complete frame found
                    this.debug && console.log('[From SP] No complete frame found, but remaining data :', remainingBuffer);

                    // save remaining data for the next flush
                    const remainData = new Uint8Array(remainingBuffer.length);
                    if (remainingBuffer.length > 0) {
                        remainData.set(remainingBuffer, 0);
                    }
                    this.buffer = remainData;

                    return callback();
                }
                if (r.bad) {
                    // bad frame, skip the bad data
                    this.buffer = remainingBuffer.subarray(r.bad!.skipEndIndex);
                    return callback();
                } else {
                    const detectedFrame = r.ok;
                    if (!detectedFrame) {
                        // never go there
                        console.error('CustomProtocolTransformFromSerialPort _flush() detectedFrame is undefined. never go there.');
                        return callback();
                    }

                    // 低4位为设备id号
                    const deviceId = detectedFrame.idField & 0x0F;
                    // 高4位为协议识别码
                    const protocolCode = detectedFrame.idField & 0xF0;
                    const protocolMode = Object.values(ProtocolMode).includes(protocolCode) ? protocolCode as ProtocolMode : ProtocolMode.Unknown;

                    const rData: CustomProtocolPackage = {
                        payload: new Uint8Array(detectedFrame.payloadSlice.length),
                        deviceId: deviceId,
                        protocolCode: protocolCode,
                        protocolMode: protocolMode,
                    };
                    rData.payload.set(detectedFrame.payloadSlice, 0);
                    this.debug && console.log('[From SP] Detected remaining frame:', rData);
                    this.push(rData);

                    if (detectedFrame.startIndex !== 0) {
                        this.debug && console.log('[From SP] Some data be trimmed before the frame start, warning.', remainingBuffer.subarray(0, detectedFrame.startIndex));
                    }

                    // remove the processed frame from the buffer
                    remainingBuffer = remainingBuffer.subarray(detectedFrame.endIndex, remainingBuffer.length - 1);
                    assert(remainingBuffer.length - (detectedFrame.startIndex + detectedFrame.frameSize) === remainingBuffer.length,
                        ' assert (remainingBuffer.length - (detectedFrame.startIndex + detectedFrame.frameSize) === remainingBuffer.length');

                    continue;
                }

            }

        }
        callback();
    }
}
