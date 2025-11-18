import moment from "moment";
import {
    AirplaneOwl02Interface,
    AirplaneOwl02State,
} from "./AirplaneOwl02Interface";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {MavLinkData, MavLinkPacket, uint8_t} from "node-mavlink";
import {MavLinkPacket2DataProcessType, PackAndDataType} from "./CustomProtocolTransformManager";
import {TimeBasedFifoCache} from "./utils/TimeBasedFifoCache";
import {BehaviorSubject, filter, firstValueFrom, Subject} from "rxjs";
import {timeoutWithoutError} from "./utils/rxjsTimeoutWithoutError";
import * as commonACFly from './Owl02Lib/commonACFly';
import * as minimalACFly from './Owl02Lib/minimalACFly';
import {getNowTimestampMsUintFloat, mathMod} from "./AirplaneTimestamp";

export interface MavLinkPacketRecord<D extends MavLinkData = MavLinkData> {
    time: moment.Moment;
    pack: MavLinkPacket;
    msgId: uint8_t;
    data?: MavLinkPacket2DataProcessType<D>;
}

function ListSwitch<I extends number | string, R>(o: I, s: Record<I, R>, d: R): R {
    // if (o in s) {
    //     return s[o];
    // }
    // return d;
    return s[o] ?? d;
}

export class AirplaneOwl02 implements AirplaneOwl02Interface {
    protected state: AirplaneOwl02State = new AirplaneOwl02State();
    protected cachedPacketRecord: Map<uint8_t, MavLinkPacketRecord> = new Map<uint8_t, MavLinkPacketRecord>();

    protected parseTable: { [key: number]: (data: PackAndDataType) => void };
    protected cachedPacketIds: Set<number>;

    protected cacheStateText: TimeBasedFifoCache<MavLinkPacketRecord<commonACFly.StatusText>> = new TimeBasedFifoCache({
        timeout: 1000 * 60 * 5,
        maxSize: 30,
    });

    public commander: AirplaneOwl02Commander;
    protected packStream = new Subject<MavLinkPacketRecord>();
    protected ackPackStream = new BehaviorSubject<PackAndDataType<commonACFly.CommandAck> | undefined>(undefined);

    constructor(
        public targetChannelId: number,
        public manager: AirplaneManagerOwl02Interface,
    ) {
        this.parseTable = {
            [minimalACFly.Heartbeat.MSG_ID]: this.parseHeartbeat.bind(this),
            [commonACFly.ExtendedSysState.MSG_ID]: this.parseLandState.bind(this),
            [commonACFly.AutopilotVersion.MSG_ID]: this.parseAutopilotVersion.bind(this),
            [commonACFly.StatusText.MSG_ID]: this.parseStatusText.bind(this),
            [commonACFly.CommandAck.MSG_ID]: this.parseAck.bind(this),
            [commonACFly.BatteryStatusAcfly.MSG_ID]: this.parseBatteryStatusAcfly.bind(this),
        };
        this.cachedPacketIds = new Set<number>([
            // 飞控解算位置
            // 报文MAVLINK_MSG_ID_GLOBAL_POSITION_INT ，ID=33，2HZ
            commonACFly.GlobalPositionInt.MSG_ID,
            commonACFly.GpsRawInt.MSG_ID,
            commonACFly.Gps2Raw.MSG_ID,
            commonACFly.VfrHud.MSG_ID,
            commonACFly.Attitude.MSG_ID,
            commonACFly.RcChannels.MSG_ID,
            commonACFly.RcChannelsScaled.MSG_ID,
            commonACFly.MissionCurrent.MSG_ID,
            commonACFly.BatteryStatus.MSG_ID,
            commonACFly.BatteryStatusAcfly.MSG_ID,
        ]);
        this.commander = new AirplaneOwl02Commander(this);
    }

    protected isInit = false;

    public async init() {
        if (this.isInit) {
            return;
        }
        this.isInit = true;

        await this.sendHeartbeat();
    }

    protected cachePacketRecord(pack: MavLinkPacket, data?: MavLinkPacket2DataProcessType): void {
        const msgId = pack.header.msgid;
        const record: MavLinkPacketRecord = {
            time: moment(),
            pack: pack,
            msgId: msgId,
            data: data,
        };
        this.cachedPacketRecord.set(msgId, record);
        this.packStream.next(record);
    }

    public sendMsg<M extends Exclude<MavLinkData, commonACFly.CommandLong>>(msg: M) {
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    public sendMsgCommand<M extends commonACFly.CommandLong>(msg: M) {
        msg.targetComponent = 1;
        msg.targetSystem = 1;
        msg._param7 = getNowTimestampMsUintFloat();
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    public async sendHeartbeat() {
        const commandHeartbeat = new minimalACFly.Heartbeat();
        commandHeartbeat.type = minimalACFly.MavType.GCS;
        commandHeartbeat.autopilot = minimalACFly.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        commandHeartbeat.baseMode = minimalACFly.MavModeFlag.CUSTOM_MODE_ENABLED;
        await this.sendMsg(commandHeartbeat);
    }

    /**
     * call this , and then the autopilot will send back AutopilotVersion pack
     */
    public async triggerGetAutopilotVersion() {
        const p = new commonACFly.RequestAutopilotCapabilitiesCommand();
        return this.sendMsgCommand(p);
    }

    protected parseHeartbeat(data: PackAndDataType) {
        const p = data.data.mavLinkData as minimalACFly.Heartbeat;
        this.state.isArmed = (p.baseMode & 0x80) === 0x80;
    }

    protected parseLandState(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.ExtendedSysState;
        this.state.isLanded = p.landedState;
    }

    protected parseStatusText(data: PackAndDataType) {
        const p = data.data as MavLinkPacket2DataProcessType<commonACFly.StatusText>;
        this.cacheStateText.push({
            time: moment(),
            pack: data.packet,
            msgId: data.packet.header.msgid,
            data: p,
        });
    }

    protected parseAutopilotVersion(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.AutopilotVersion;
    }

    protected parseAck(data: PackAndDataType) {
        this.ackPackStream.next(data as PackAndDataType<commonACFly.CommandAck>);
        const p = data.data.mavLinkData as commonACFly.CommandAck;
        // TODO
        console.log('[AirplaneOwl02] CommandAck:', p);
        const cmdId = p.command;
        const isOk = p.result === 0;
        const ackPackTimestamp = p.resultParam2;
        const progress = p.progress;
    }

    protected parseBatteryStatusAcfly(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.BatteryStatusAcfly;
        // 电池电压，单位100mv
        // 无效值: 0xFFFFFFFF
        p.voltage;
        // 电池容量，单位mAh
        // 无效值: 0xFFFFFFFF
        p.capacity;
        // 电池序列号
        // 无效值: 0xFFFFFFFF
        p.sequenceNum;
        // 错误标记位
        // 无效值0xFF
        p.faultBitmask;
        // 温度，单位度
        // 无效值0x7FFF
        p.temperature;
        // 循环次数
        // 无效值0xFFFF
        p.cycleCount;
        // 电量，单位100mA
        p.current;
        // 电池id
        // 根据此字段区分多个电池
        p.id;
        // 健康度，[0 - 100]
        p.health;
        // 剩余百分比 [0 - 100]
        p.remainingPercentage;
        // 电池节数，无效值0xFF
        // 注:航模电池一般由多节电池组成
        p.cellCount;
        // 单节电池电压，一般满电为4.2V；单位0.02mV，无效值0x00
        // 例voltages[0]=210，那么第一节电池电压为210*0.02=4.2V
        p.voltages;
    }

    public async parseStateFromMavLink(data: PackAndDataType) {
        this.cachePacketRecord(data.packet, data.data);
        const id = data.packet.header.msgid;
        const func = this.parseTable[id];
        if (func) {
            func(data);
        } else {
            if (!this.cachedPacketIds.has(id)) {
                console.warn('[AirplaneOwl02] Unknow pack: ', id, data);
            }
        }
    }

    public getGpsPos() {
        const p = this.cachedPacketRecord.get(commonACFly.GlobalPositionInt.MSG_ID) as undefined | commonACFly.GlobalPositionInt;
        if (!p) {
            return undefined;
        }
        return {
            lat: p.lat / 1e7,
            lon: p.lon / 1e7,
            alt: p.alt / 1e4,
            relativeAlt: p.relativeAlt / 1e4,
            vx: p.vx,
            vy: p.vy,
            vz: p.vz,
            hdg: p.hdg,
        }
    }

    public getAttitude() {
        const p = this.cachedPacketRecord.get(commonACFly.Attitude.MSG_ID) as undefined | commonACFly.Attitude;
        if (!p) {
            return undefined;
        }
        return {
            roll: p.roll,
            pitch: p.pitch,
            yaw: p.yaw,
            rollSpeed: p.rollspeed,
            pitchSpeed: p.pitchspeed,
            yawSpeed: p.yawspeed,
            timeBootMs: p.timeBootMs,
        };
    }

    public getStatusTextList(): { text: string, time: moment.Moment, severity: number }[] {
        return this.cacheStateText.toArray().map(T => {
            return {
                text: T.data?.mavLinkData.text ?? '',
                time: T.time,
                severity: T.data?.mavLinkData.severity ?? 0,
            };
        });
    }

    public waitAckOnce(timeoutMs: number, command: commonACFly.MavCmd) {
        return firstValueFrom(this.ackPackStream.pipe(
            filter((T): T is PackAndDataType<commonACFly.CommandAck> => !!T),
            filter(P => P.data.mavLinkData.command === command),
            timeoutWithoutError(timeoutMs, undefined),
        ));
    }

    public destroy() {
        this.packStream.complete();
        this.ackPackStream.complete();
        this.cacheStateText.clear();
        this.cachedPacketRecord.clear();
    }

}

export class AirplaneOwl02Commander {

    constructor(
        public airplane: AirplaneOwl02,
    ) {
    }

    lock() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 0;
        return this.airplane.sendMsgCommand(p);
    }

    unlock() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 1;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * @param height m
     */
    takeoff(height: number) {
        const p = new commonACFly.ExtDroneTakeoffCommand();
        p.height = height;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * 一键降落
     */
    land() {
        const p = new commonACFly.ExtDroneLandCommand();
        p.land_mode = 1;
        p.landspeed = 0;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * 室内定点飞行
     * @param x       （x，cm）
     * @param y       （y，cm）
     * @param h       （高度，cm）
     */
    goto(x: number, y: number, h: number) {
        const p = new commonACFly.ExtDroneGotoCmdCommand();
        p.target_x = x;
        p.target_y = y;
        p.target_z = h;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * @param forward 1：上升 2：下降，3：前，4：后，5：左，6：右
     * @param distance
     * @param speed
     */
    move(forward: 1 | 2 | 3 | 4 | 5 | 6, distance: number, speed: number = 0) {
        const p = new commonACFly.ExtDroneMoveCommand();
        p.direction = 5;
        p.distance = distance;
        p.speed = speed;
        return this.airplane.sendMsgCommand(p);
    }

    up(distance: number, speed: number = 0) {
        return this.move(1, distance, speed);
    }

    down(distance: number, speed: number = 0) {
        return this.move(2, distance, speed);
    }

    forward(distance: number, speed: number = 0) {
        return this.move(3, distance, speed);
    }

    back(distance: number, speed: number = 0) {
        return this.move(4, distance, speed);
    }

    left(distance: number, speed: number = 0) {
        return this.move(5, distance, speed);
    }

    right(distance: number, speed: number = 0) {
        return this.move(6, distance, speed);
    }

    /**
     * @param forward 1：逆时针 2：顺时针
     * @param degrees  转动角度（单位 度，min值0，max值360）
     */
    rotate(forward: 1 | 2, degrees: number) {
        // mod(degrees,360)
        degrees = mathMod(degrees, 360);
        const p = new commonACFly.ExtDroneCircleCommand();
        p.direction = forward;
        p.degrees = degrees;
        return this.airplane.sendMsgCommand(p);
    }

    cw(degrees: number) {
        return this.rotate(2, degrees);
    }

    ccw(degrees: number) {
        return this.rotate(1, degrees);
    }

    setSpeed(speed: number) {
        const p = new commonACFly.ExtDroneChangeSpeedCommand();
        p.speed = speed;
        return this.airplane.sendMsgCommand(p);
    }

    light(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 0;
        return this.airplane.sendMsgCommand(p);
    }

    rainbow(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 1;
        return this.airplane.sendMsgCommand(p);
    }

    breathe(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 1;
        p.rainbow = 0;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Return to launch - RTL command
     */
    returnToLaunch() {
        const p = new commonACFly.NavReturnToLaunchCommand();
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Set altitude - simplified version of goto
     * @param high altitude in cm
     */
    high(high: number) {
        const p = new commonACFly.ExtDroneSetHeghtCommand();
        p.height = high;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Set flight mode
     * @param mode 1:normal 2:track line 3:follow
     */
    setMode(mode: 1 | 2 | 3) {
        const p = new commonACFly.ExtDroneSetModeCommand();
        p.mode = mode;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Set color detection mode
     * @param l_min L channel min value
     * @param l_max L channel max value
     * @param a_min A channel min value
     * @param a_max A channel max value
     * @param b_min B channel min value
     * @param b_max B channel max value
     */
    setColorDetectMode(l_min: number, l_max: number, a_min: number, a_max: number, b_min: number, b_max: number) {
        const p = new commonACFly.ExtDroneVisionDetectModeSetCommand();
        p.l_l = l_min;
        p.l_h = l_max;
        p.a_l = a_min;
        p.a_h = a_max;
        p.b_l = b_min;
        p.b_h = b_max;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Emergency stop - stop motors immediately
     */
    emergency_stop() {
        const p = new commonACFly.ExtDroneUrgentDisarmCommand();
        p.cmd = 1; // disarm
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Hover in place - stop current movement
     */
    hover() {
        const p = new commonACFly.ExtDroneHoverCommand();
        p.cmd = 1;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Flip forward
     */
    flipForward() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 1; // forward
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Flip backward
     */
    flipBack() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 2; // backward
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Flip left
     */
    flipLeft() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 3; // left
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * Flip right
     */
    flipRight() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 4; // right
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * 设置openmv识别模式
     * @param mode (1常规 2巡线 3跟随)
     */
    set_openmv_mode(mode: number) {
        const p = new commonACFly.ExtDroneSetModeCommand();
        p.mode = mode;
        return this.airplane.sendMsgCommand(p);
    }

    /**
     * 开始OPENMV相关运动 MAV_CMD_EXT_DRONE_OPEMMV_CMD
     * 开启之前要通过 MAV_CMD_EXT_DRONE_SET_MODE / MAV_CMD_EXT_DRONE_VISION_DETECT_MODE_SET 对应的识别模式和设置
     * @param cmd  视觉模式值 (0:巡线 1:锁定二维码，飞到二维码正上方，3：寻找色块)
     * @param x    x轴移动距离
     * @param y    y轴移动距离
     * @param z    z轴移动距离
     */
    go_openmv_cmd(cmd: number, x: number, y: number, z: number) {
        const p = new commonACFly.ExtDroneOpemmvCmdCommand();
        p.cmd = cmd;
        p.xDistance = x;
        p.yDistance = y;
        p.zDistance = z;
        return this.airplane.sendMsgCommand(p);
    }

}
