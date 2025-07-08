import {
    ardupilotmega,
    common, MavLinkPacket,
    MavLinkPacketParser,
    MavLinkPacketRegistry,
    MavLinkPacketSplitter,
    MavLinkProtocolV2,
    minimal,
} from 'node-mavlink';
import {Duplex, PassThrough, Transform, TransformCallback, Stream, Writable} from "stream";

// create a registry of mappings between a message id and a data class
// 注册表，用于将消息ID映射到数据类
const REGISTRY: MavLinkPacketRegistry = {
    ...minimal.REGISTRY,
    ...common.REGISTRY,
    ...ardupilotmega.REGISTRY,
};

function hexStringToBuffer(hexString: string): Buffer {

    // 移除所有空格
    const cleanHex = hexString.replace(/\s+/g, '');

    // 检查是否为有效的十六进制字符串
    if (!/^[0-9A-Fa-f]*$/.test(cleanHex)) {
        throw new Error('Invalid hex string');
    }

    // 确保字符串长度为偶数
    if (cleanHex.length % 2 !== 0) {
        throw new Error('Hex string must have even length');
    }

    // 转换为Buffer
    const b = Buffer.from(cleanHex, 'hex');
    console.log('Buffer:', b);
    return b;
}

async function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

;(async () => {
    const protocol = new MavLinkProtocolV2(1, 1);

    const mavLinkDecodeStream: PassThrough = new PassThrough();
    const outputStream = mavLinkDecodeStream
        .pipe(new MavLinkPacketSplitter({}, {
            onCrcError: E => console.log('Crc error:', E),
        }))
        .pipe(new MavLinkPacketParser());
    outputStream.on('data', (packet: MavLinkPacket) => {
        // console.log('[test1] Received packet:', packet.header);
        const clazz = REGISTRY[packet.header.msgid];
        if (clazz) {
            const data = packet.protocol.data(packet.payload, clazz);
            console.log('[test1] Received data:', packet.header, data);
        } else {
            console.warn('[test1] Cannot decode packet :', packet.header.msgid, packet);
        }
    });

    // const cl = new common.CommandLong();
    // cl.targetSystem = 0;
    // cl.targetComponent = 0;
    // // cl.command = common.MavCmd.COMPONENT_ARM_DISARM;
    // // @ts-ignore
    // cl.command = 1;
    // cl._param1 = 0;
    // console.log(cl);
    // const b1 = protocol.serialize(cl, 0);

    const unlock = new common.ComponentArmDisarmCommand(1, 1);
    unlock.arm = 1;
    unlock.arm = 0;
    unlock.force = 1;
    unlock.targetSystem = 1;
    unlock.targetComponent = 1;
    console.log(unlock);
    const b1 = protocol.serialize(unlock, 234);

    console.log('[test1] Serialized CommandLong b1:', b1);
    await sleep(100);
    mavLinkDecodeStream.push(b1);
    // <Buffer fd 20 00 00 00 fe 01 4c 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 90 01 01 01 b6 fc>
    // <Buffer fd 1d 00 00 ea 01 01 4c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 8d d7>

    await sleep(100);
    mavLinkDecodeStream.push(hexStringToBuffer(
        'FD 1E 00 00 B3 01 01 4C 00 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 90 01 77 CE'
    ));
    // await sleep(100);
    // mavLinkDecodeStream.push(hexStringToBuffer(
    //     'FD 1D 00 00 EA 01 01 4C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 8D D7'
    // ));
    // await sleep(100);
    // mavLinkDecodeStream.push(hexStringToBuffer(
    //     'FD 1D 00 00 ED 01 01 4C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 53 8A'
    // ));
    // await sleep(100);
    // mavLinkDecodeStream.push(hexStringToBuffer(
    //     'FD 1D 00 00 EF 01 01 4C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 DC DA'
    // ));

})().catch(console.error);

