import {SerialPort} from 'serialport';
import {
    ardupilotmega,
    common,
    MavLinkPacketRegistry,
    MavLinkProtocolV2,
    minimal,
} from 'node-mavlink';
import {CustomProtocolTransformManager} from "./CustomProtocolTransformManager";
import {MavStateCollector} from "./MavStateCollector";

// 替换 COM3 为你的串口路径，或从环境变量设置，可以使用 UsbTreeView 或从设备管理中查看当前所有串口
const comPortString = process.env.COM_PORT_STRING || 'COM10';

// console.log('process.env', process.env);
console.log('comPortString', comPortString);

console.log('Hello World ✨');

// create a registry of mappings between a message id and a data class
// 注册表，用于将消息ID映射到数据类
const REGISTRY: MavLinkPacketRegistry = {
    ...minimal.REGISTRY,
    ...common.REGISTRY,
    ...ardupilotmega.REGISTRY,
};

// 初始化 SerialPort 实例
const port = new SerialPort({path: comPortString, baudRate: 921600});

const m = new CustomProtocolTransformManager(port, new MavLinkProtocolV2(1, 1), REGISTRY);

// 接收id 0 的数据
m.getDecoderStream(0).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        console.log('====== Received data [0]:', data);
    },
});
// 接收id 1 的数据
m.getDecoderStream(1).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        console.log('====== Received data [1]:', data);
    },
});
// 接收id 2 的数据
m.getDecoderStream(2).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        console.log('====== Received data [2]:', data);
    },
});
// 接收id 3 的数据
m.getDecoderStream(3).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        console.log('====== Received data [3]:', data);
    },
});
m.getMavLinkAllDataObservable().subscribe({
    next: (data) => {
        // 处理接收到的所有数据
        console.log('====== Received data [all]:', data);
    },
});

const mavStateCollector = new MavStateCollector(m);
// mavStateCollector.getStateInfo(0);


// sleep 函数，可用在下面的代码中来模拟延时操作
async function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

port.on('open', async () => {
    // the port is open - we're ready to send data
    // 串口已打开 - 准备发送数据

    // 构造一个心跳包，填充数据并发送
    // console.log('====== commandHeartbeat');
    // const commandHeartbeat = new minimal.Heartbeat();
    // commandHeartbeat.systemStatus = minimal.MavState.STANDBY;
    // // 将心跳包发送到设备ID 0
    // await m.sendMsg(commandHeartbeat, 0);
    // commandHeartbeat.systemStatus = minimal.MavState.BOOT;
    // // 将心跳包发送到设备ID 1
    // await m.sendMsg(commandHeartbeat, 1);
    //
    // commandHeartbeat.systemStatus = minimal.MavState.POWEROFF;
    // // 将心跳包发送到设备ID 3
    // // await m.sendMsg(commandHeartbeat, 3);
    //
    // // 模拟等待一段时间
    // await sleep(500);
    //
    // // 构造一个系统时间包，填充数据并发送
    // console.log('====== commandSystemTime');
    // const commandSystemTime = new common.SystemTime();
    // commandSystemTime.timeUnixUsec = BigInt(Date.now()) * BigInt(1000);
    // commandSystemTime.timeBootMs = process.uptime() * 1000;
    // // 将包发送到设备ID 0
    // await m.sendMsg(commandSystemTime, 0);
    // await sleep(500);
    // commandSystemTime.timeUnixUsec = BigInt(Date.now()) * BigInt(1000);
    // commandSystemTime.timeBootMs = process.uptime() * 1000;
    // // 将包发送到设备ID 1
    // await m.sendMsg(commandSystemTime, 1);
    // // await m.sendMsg(commandSystemTime, 3);

    // console.log('====== ComponentArmDisarmCommand');
    // console.log('common.ComponentArmDisarmCommand.PAYLOAD_LENGTH', common.ComponentArmDisarmCommand.PAYLOAD_LENGTH);
    // const unlock = new common.ComponentArmDisarmCommand(1, 1);
    // unlock.arm = 1;
    // unlock.arm = 0;
    // unlock.force = 1;
    // unlock.targetSystem = 1;
    // unlock.targetComponent = 1;
    // console.log(unlock)
    // await m.sendMsg(unlock, 0);
    // await m.sendMsg(unlock, 1);
    // await m.sendMsg(unlock, 2);

    console.log('====== CommandLong COMPONENT_ARM_DISARM');
    const cl = new common.CommandLong();
    cl.targetSystem = 0;
    cl.targetComponent = 0;
    cl.command = common.MavCmd.COMPONENT_ARM_DISARM;
    cl._param1 = 1;
    console.log(cl);

    // const cl = new common.CommandLong();
    // cl.targetSystem = 1;
    // cl.targetComponent = 1;
    // // cl.command = common.MavCmd.COMPONENT_ARM_DISARM;
    // // @ts-ignore
    // cl.command = 1;
    // cl._param1 = 1;
    // console.log(cl);
    // await m.sendMsg(cl, 0);
    await m.sendMsg(cl, 0);
    // await m.sendMsg(cl, 1);
    // await m.sendMsg(cl, 1);
    // await m.sendMsg(cl, 2);

    console.log('====== sendEnd');
});

