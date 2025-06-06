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
    MavLinkPacket,
} from 'node-mavlink';
import {PassThrough} from 'stream';
import {CustomProtocolTransformFromSerialPort, CustomProtocolTransformToSerialPort} from "./CustomProtocolTransform";

console.log('Hello World ✨');

// create a registry of mappings between a message id and a data class
// 注册表，用于将消息ID映射到数据类
const REGISTRY: MavLinkPacketRegistry = {
    ...minimal.REGISTRY,
    ...common.REGISTRY,
    ...ardupilotmega.REGISTRY,
};

// 初始化 SerialPort 实例，替换 path 为你的串口路径，可以使用 UsbTreeView 或从设备管理中查看当前所有串口
const port = new SerialPort({path: 'COM4', baudRate: 115200});

// from SerialPort
// 创建一个 PassThrough 流，用于从 SerialPort 读取数据，并加入自定义协议解析器
const customProtocolReadStream = new PassThrough();
port.pipe(new CustomProtocolTransformFromSerialPort(false)).pipe(customProtocolReadStream)

// to SerialPort
// 创建一个 PassThrough 流，用于向 SerialPort 写入数据，并加入自定义协议封装器
const customProtocolWriteStream = new PassThrough();
customProtocolWriteStream.pipe(new CustomProtocolTransformToSerialPort(false)).pipe(port);

// constructing a reader that will emit each packet separately
// 在读取流中添加MavLink包切分器和包解析器来将二进制流转换为 MavLink 数据包对象流
const reader = customProtocolReadStream
    .pipe(new MavLinkPacketSplitter())
    .pipe(new MavLinkPacketParser());

// 监听从串口读取到并解包后的每个 MavLink 数据包
reader.on('data', (packet: MavLinkPacket) => {
    console.log('====== reader received a pack, wait to parse it: ', packet)

    // 使用注册表将消息ID映射到数据类，并使用协议解析器将负载数据转换为相应的数据对象
    const clazz = REGISTRY[packet.header.msgid];
    if (clazz) {
        const data = packet.protocol.data(packet.payload, clazz);
        console.log(`====== Received packet [${clazz.MSG_NAME}] :`, data);
    }
});

// sleep 函数，可用在下面的代码中来模拟延时操作
async function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

port.on('open', async () => {
    // the port is open - we're ready to send data
    // 串口已打开 - 准备发送数据

    // 构造一个心跳包，填充数据并发送
    console.log('====== commandHeartbeat');
    const commandHeartbeat = new minimal.Heartbeat();
    commandHeartbeat.systemStatus = minimal.MavState.STANDBY;
    await send(customProtocolWriteStream, commandHeartbeat, new MavLinkProtocolV2());

    // 模拟等待一段时间
    await sleep(500);

    // 构造一个系统时间包，填充数据并发送
    console.log('====== commandSystemTime');
    const commandSystemTime = new common.SystemTime();
    commandSystemTime.timeUnixUsec = BigInt(Date.now()) * BigInt(1000);
    commandSystemTime.timeBootMs = process.uptime() * 1000;
    await send(customProtocolWriteStream, commandSystemTime, new MavLinkProtocolV2());

    console.log('====== sendEnd');
});

