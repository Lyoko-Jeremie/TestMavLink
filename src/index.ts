import {SerialPort} from 'serialport';
import {
    MavLinkData,
    MavLinkProtocolV2,
} from 'node-mavlink';
import {CustomProtocolTransformManager} from "./CustomProtocolTransformManager";
import {MavStateCollector} from "./MavStateCollector";
import {UtilTimer} from "./utils/UtilTimer";
import * as commonACFly from "./Owl02Lib/commonACFly";
import * as minimalACFly from "./Owl02Lib/minimalACFly";
import {getNowTimestampMsUintFloat} from "./AirplaneTimestamp";
import {REGISTRY} from "./Owl02Lib/commonACFly";
import {AirplaneManagerOwl02} from "./AirplaneManagerOwl02";

// 替换 COM3 为你的串口路径，或从环境变量设置，可以使用 UsbTreeView 或从设备管理中查看当前所有串口
const comPortString = process.env.COM_PORT_STRING || 'COM10';

// console.log('process.env', process.env);
console.log('comPortString', comPortString);

console.log('Hello World ✨');

// 初始化 SerialPort 实例
const port = new SerialPort({path: comPortString, baudRate: 921600});

const m = new CustomProtocolTransformManager(port, new MavLinkProtocolV2(1, 1), REGISTRY, false);

// 接收id 0 的数据
m.getDecoderStream(0).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        // console.log('====== Received data [0]:', data);
    },
});
// 接收id 1 的数据
m.getDecoderStream(1).observableData.subscribe({
    next: (data: MavLinkData) => {
        // 处理接收到的数据
        // console.log('====== Received data [1]:', data);
        // console.log('====== Received data [1]:', data);
    },
});
// 接收id 2 的数据
let sq: string[] = [];
m.getDecoderStream(2).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        // console.log('====== Received data [2]:', data);
        // console.log('====== Received data [2]:', data.MSG_NAME);
        // switch (data.MSG_NAME) {
        //     case 'HEARTBEAT':
        //         // sq.push('.');
        //         process.stdout.write('.');
        //         break;
        //     case 'BATTERY_STATUS':
        //         // sq.push('-');
        //         process.stdout.write('-');
        //         break;
        //     case 'SYS_STATUS':
        //         // sq.push('*');
        //         process.stdout.write('*');
        //         break;
        //     case 'SYSTEM_TIME':
        //         // sq.push('/');
        //         process.stdout.write('/');
        //         break;
        //     case 'RC_CHANNELS':
        //         // sq.push('|');
        //         process.stdout.write('|');
        //         break;
        //     case 'ATTITUDE':
        //         // Attitude {
        //         //     timeBootMs: 2291140,
        //         //     roll: 0.017909057438373566,
        //         //     pitch: 0.004482499789446592,
        //         //     yaw: 2.1127374172210693,
        //         //     rollspeed: -0.002332122763618827,
        //         //     pitchspeed: 0.0029339175671339035,
        //         //     yawspeed: 0.0014431369490921497
        //         // }
        //         process.stdout.write('|');
        //         break;
        //     default:
        //         // sq.push('?');
        //         // console.log(sq.join(''));
        //         // sq = [];
        //         process.stdout.write('\n');
        //         console.log('====== Received data [2]:', data);
        // }
        // console.log('====== Received data [2]:');
        // console.log(JSON.stringify(data))
    },
});
// 接收id 3 的数据
m.getDecoderStream(3).observableData.subscribe({
    next: (data) => {
        // 处理接收到的数据
        // console.log('====== Received data [3]:', data);
    },
});
m.getMavLinkAllDataObservable().subscribe({
    next: (data) => {
        // 处理接收到的所有数据
        // console.log('====== Received data [all]:', data);

        // CommandAck

        switch (data.data.MSG_NAME) {
            case 'HEARTBEAT':
                // sq.push('.');
                process.stdout.write(`${data.id}.`);
                break;
            case 'BATTERY_STATUS':
                // sq.push('-');
                process.stdout.write(`${data.id}-`);
                break;
            case 'SYS_STATUS':
                // sq.push('*');
                process.stdout.write(`${data.id}*`);
                break;
            case 'SYSTEM_TIME':
                // sq.push('/');
                process.stdout.write(`${data.id}/`);
                break;
            case 'RC_CHANNELS':
                // sq.push('|');
                process.stdout.write(`${data.id}|`);
                break;
            case 'ATTITUDE':
                // Attitude {
                //     timeBootMs: 2291140,
                //     roll: 0.017909057438373566,
                //     pitch: 0.004482499789446592,
                //     yaw: 2.1127374172210693,
                //     rollspeed: -0.002332122763618827,
                //     pitchspeed: 0.0029339175671339035,
                //     yawspeed: 0.0014431369490921497
                // }
                process.stdout.write(`${data.id}$`);
                break;
            case 'COMMAND_ACK':
                break;
            default:
                // sq.push('?');
                // console.log(sq.join(''));
                // sq = [];
                process.stdout.write('\ndata.data.MSG_NAME\n');
                console.log(`====== Received data [${data.id}]:`, data);
        }
    },
});

const mavStateCollector = new MavStateCollector(m);
// mavStateCollector.getStateInfo(0);


// sleep 函数，可用在下面的代码中来模拟延时操作
async function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

const heartbeatTimer = new UtilTimer(
    async () => {
        const commandHeartbeat = new minimalACFly.Heartbeat();
        commandHeartbeat.type = minimalACFly.MavType.GCS;
        commandHeartbeat.autopilot = minimalACFly.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        commandHeartbeat.baseMode = minimalACFly.MavModeFlag.CUSTOM_MODE_ENABLED;
        // custom_mode的第3个字节代表飞机的主模式，定义如下
        // 定高模式（2）
        // 定点模式（3）
        // 自动模式（4）
        // custom_mode的第4个字节代表飞机的细分模式，定义如下
        // 自动模式细分：
        // 	自动起飞模式（2）
        // 	自动跟踪模式（3）
        // 	自动任务模式（4）
        // 	自动返航模式（5）
        // 	自动降落模式（6）
        // 定点模式细分：
        // 	普通定点模式（0）
        // 	定点避障模式（2）
        // commandHeartbeat.customMode = ;
        commandHeartbeat.systemStatus = minimalACFly.MavState.ACTIVE;
    },
    console,
    500,
);

const flyManager = new AirplaneManagerOwl02(m);

port.on('open', async () => {
    // the port is open - we're ready to send data
    // 串口已打开 - 准备发送数据

    // heartbeatTimer.start();

    // // 构造一个心跳包，填充数据并发送
    // console.log('====== commandHeartbeat');
    // const commandHeartbeat = new minimalACFly.Heartbeat();
    // commandHeartbeat.systemStatus = minimalACFly.MavState.STANDBY;
    // // console.log(m.debugSerializeMavLinkMsg(commandHeartbeat));
    // // 将心跳包发送到设备ID 0
    // await m.sendMsg(commandHeartbeat, 0);
    // // commandHeartbeat.systemStatus = minimalACFly.MavState.BOOT;
    // // 将心跳包发送到设备ID 1
    // await m.sendMsg(commandHeartbeat, 1);
    // //
    // // commandHeartbeat.systemStatus = minimalACFly.MavState.POWEROFF;
    // // 将心跳包发送到设备ID 3
    // await m.sendMsg(commandHeartbeat, 3);
    //
    // 模拟等待一段时间
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
    // // console.log('common.ComponentArmDisarmCommand.PAYLOAD_LENGTH', common.ComponentArmDisarmCommand.PAYLOAD_LENGTH);
    // const unlock = new common.ComponentArmDisarmCommand(1, 1);
    // // unlock.arm = 1;
    // unlock.arm = 0;
    // // unlock.force = 1;
    // unlock.targetSystem = 1;
    // unlock.targetComponent = 1;
    // // unlock.targetSystem = 0;
    // // unlock.targetComponent = 0;
    // console.log(unlock);
    // // await m.sendMsg(unlock, 0);
    // // await m.sendMsg(unlock, 1);
    // await sleep(1000);
    // await m.sendMsg(unlock, 2);
    //
    // unlock.arm = 1;
    // await sleep(3000);
    // await m.sendMsg(unlock, 2);
    //
    // unlock.arm = 0;
    // await sleep(3000);
    // await m.sendMsg(unlock, 2);
    //
    //
    // await sleep(1000);
    // await m.sendMsg(unlock, 1);
    //
    // unlock.arm = 1;
    // await sleep(3000);
    // await m.sendMsg(unlock, 1);
    //
    // unlock.arm = 0;
    // await sleep(3000);
    // await m.sendMsg(unlock, 1);

    // console.log('====== CommandLong COMPONENT_ARM_DISARM');
    // const cl = new common.CommandLong();
    // cl.targetSystem = 0;
    // cl.targetComponent = 0;
    // cl.command = common.MavCmd.COMPONENT_ARM_DISARM;
    // cl._param1 = 0;
    // // cl._param1 = 1;
    // console.log(cl);

    // const cl = new common.CommandLong();
    // cl.targetSystem = 1;
    // cl.targetComponent = 1;
    // // cl.command = common.MavCmd.COMPONENT_ARM_DISARM;
    // // @ts-ignore
    // cl.command = 1;
    // cl._param1 = 1;
    // console.log(cl);
    // await m.sendMsg(cl, 0);
    // await m.sendMsg(cl, 0);
    // await m.sendMsg(cl, 1);
    // await m.sendMsg(cl, 1);
    // await m.sendMsg(cl, 2);
    // await m.sendMsg(cl, 2);
    // for (let i = 0; i <= 16; i++) {
    //     // await m.sendMsg(cl, i);
    //     for (let j = 0; j < 1; j++) {
    //         await m.sendMsg(unlock, i);
    //         await sleep(1);
    //     }
    //     // await m.sendMsg(cl, i);
    //     await sleep(50);
    // }

    // console.log('====== ExtDroneMoveCommand');
    // const move = new commonACFly.ExtDroneMoveCommand();
    // move.direction = 1;
    // move.distance = 100;
    // move._param7 = getNowTimestampMsUintFloat();
    // await m.sendMsg(move, 2);

    // console.log('====== ExtDroneGotoCmdCommand');
    // const go = new commonACFly.ExtDroneGotoCmdCommand();
    // go.target_x = 100;
    // go.target_y = 100;
    // go.target_z = 100;
    // go._param7 = getNowTimestampMsUintFloat();     // timestamp
    // await m.sendMsg(go, 2);

    // console.log('====== ExtDroneTakeoffCommand');
    // const takeoff = new commonACFly.ExtDroneTakeoffCommand();
    // takeoff.height = 100;
    // await m.sendMsg(takeoff, 2);


    console.log('====== init flyManager');

    flyManager.init();
    await sleep(1000);

    // id 0 === airplane 1
    const f = await flyManager.getAirplane(2);

    // change: 需要修改为，收到takeoff自动解锁
    await f.commander.unlock();
    await sleep(3000);

    // bug: ack 有延迟。回程消息步率不稳定。
    await f.commander.takeoff(100);
    await sleep(8000);

    await f.commander.back(50);
    await sleep(3000);
    // bug: 上一个动作没做完，会忽略新的命令
    await f.commander.forward(50);
    await sleep(3000);

    // notice: lock 指令无效，需使用land替代
    // await f.commander.lock();
    // await sleep(2000);

    // bug: 飞行速度较慢
    await f.commander.land();
    await sleep(5000);

    console.log('====== sendEnd');
});

