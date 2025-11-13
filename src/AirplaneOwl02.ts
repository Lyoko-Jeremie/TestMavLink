import moment from "moment";
import {
    AirplaneOwl02Interface,
    AirplaneOwl02State,
} from "./AirplaneOwl02Interface";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {common, MavLinkData, MavLinkPacket, minimal, uint8_t} from "node-mavlink";
import {MavLinkPacket2DataProcessType, PackAndDataType} from "./CustomProtocolTransformManager";
import {TimeBasedFifoCache} from "./utils/TimeBasedFifoCache";
import {BehaviorSubject, filter, firstValueFrom, Subject} from "rxjs";
import {timeoutWithoutError} from "./utils/rxjsTimeoutWithoutError";
import * as commonACFly from './Owl02Lib/commonACFly';
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

    protected cacheStateText: TimeBasedFifoCache<MavLinkPacketRecord<common.StatusText>> = new TimeBasedFifoCache({
        timeout: 1000 * 60 * 5,
        maxSize: 30,
    });

    public commander: AirplaneOwl02Commander;
    protected packStream = new Subject<MavLinkPacketRecord>();
    protected ackPackStream = new BehaviorSubject<PackAndDataType<common.CommandAck> | undefined>(undefined);

    constructor(
        public targetChannelId: number,
        public manager: AirplaneManagerOwl02Interface,
    ) {
        this.parseTable = {
            [minimal.Heartbeat.MSG_ID]: this.parseHeartbeat.bind(this),
            [common.ExtendedSysState.MSG_ID]: this.parseLandState.bind(this),
            [common.AutopilotVersion.MSG_ID]: this.parseAutopilotVersion.bind(this),
            [common.StatusText.MSG_ID]: this.parseStatusText.bind(this),
            [common.CommandAck.MSG_ID]: this.parseAck.bind(this),
            [commonACFly.BatteryStatusAcfly.MSG_ID]: this.parseBatteryStatusAcfly.bind(this),
        };
        this.cachedPacketIds = new Set<number>([
            // 飞控解算位置
            // 报文MAVLINK_MSG_ID_GLOBAL_POSITION_INT ，ID=33，2HZ
            common.GlobalPositionInt.MSG_ID,
            common.GpsRawInt.MSG_ID,
            common.Gps2Raw.MSG_ID,
            common.VfrHud.MSG_ID,
            common.Attitude.MSG_ID,
            common.RcChannels.MSG_ID,
            common.RcChannelsScaled.MSG_ID,
            common.MissionCurrent.MSG_ID,
            common.BatteryStatus.MSG_ID,
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

    public sendMsg(msg: MavLinkData) {
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    public async sendHeartbeat() {
        const commandHeartbeat = new minimal.Heartbeat();
        commandHeartbeat.type = minimal.MavType.GCS;
        commandHeartbeat.autopilot = minimal.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        commandHeartbeat.baseMode = minimal.MavModeFlag.CUSTOM_MODE_ENABLED;
        await this.sendMsg(commandHeartbeat);
    }

    /**
     * call this , and then the autopilot will send back AutopilotVersion pack
     */
    public async triggerGetAutopilotVersion() {
        const p = new common.RequestAutopilotCapabilitiesCommand();
        p.targetComponent = 1;
        p.targetComponent = 1;
        return await this.sendMsg(p);
    }

    protected parseHeartbeat(data: PackAndDataType) {
        const p = data.data.mavLinkData as minimal.Heartbeat;
        this.state.isArmed = (p.baseMode & 0x80) === 0x80;
    }

    protected parseLandState(data: PackAndDataType) {
        const p = data.data.mavLinkData as common.ExtendedSysState;
        this.state.isLanded = p.landedState;
    }

    protected parseStatusText(data: PackAndDataType) {
        const p = data.data as MavLinkPacket2DataProcessType<common.StatusText>;
        this.cacheStateText.push({
            time: moment(),
            pack: data.packet,
            msgId: data.packet.header.msgid,
            data: p,
        });
    }

    protected parseAutopilotVersion(data: PackAndDataType) {
        const p = data.data.mavLinkData as common.AutopilotVersion;
    }

    protected parseAck(data: PackAndDataType) {
        this.ackPackStream.next(data as PackAndDataType<common.CommandAck>);
        const p = data.data.mavLinkData as common.CommandAck;
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
        const p = this.cachedPacketRecord.get(common.GlobalPositionInt.MSG_ID) as undefined | common.GlobalPositionInt;
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
        const p = this.cachedPacketRecord.get(common.Attitude.MSG_ID) as undefined | common.Attitude;
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

    public waitAckOnce(timeoutMs: number, command: common.MavCmd) {
        return firstValueFrom(this.ackPackStream.pipe(
            filter((T): T is PackAndDataType<common.CommandAck> => !!T),
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
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    unlock() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 1;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * @param height m
     */
    takeoff(height: number) {
        const p = new commonACFly.ExtDroneTakeoffCommand();
        p.height = height;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * 一键降落
     * @param landHeight    降落高度(相对起飞点)，单位m
     * @param yawAngle      飞机航向，单位度
     */
    land() {
        const p = new commonACFly.ExtDroneLandCommand();
        p.land_mode = 1;
        p.landspeed = 0;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * 室内定点飞行
     * @param x       （x，cm）
     * @param y       （y，cm）
     * @param h       （高度，cm）
     * @param yawAngle  （航向角，-360 ~360，填nan则指向航点再飞过去， 填1000则不旋转偏航）
     * @param speed     （速度，m/s）
     */
    goto(x: number, y: number, h: number, yawAngle: number, speed: number) {
        const p = new commonACFly.ExtDroneGotoCmdCommand();
        p.target_x = x;
        p.target_y = y;
        p.target_z = h;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
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
        p.relative_or_absolute = 0;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
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
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
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
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    light(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 0;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    rainbow(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 1;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    breathe(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 1;
        p.rainbow = 0;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Return to launch - RTL command
     */
    returnToLaunch() {
        const p = new commonACFly.NavReturnToLaunchCommand();
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Set altitude - simplified version of goto
     * @param high altitude in cm
     * @param speed
     */
    high(high: number, speed: number = 0) {
        const p = new commonACFly.ExtDroneMoveCommand();
        p.direction = 1;
        p.distance = high;
        p.speed = speed;
        p.relative_or_absolute = 1;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Set flight mode
     * @param mode 1:normal 2:track line 3:follow
     */
    setMode(mode: 1 | 2 | 3) {
        const p = new commonACFly.ExtDroneSetModeCommand();
        p.mode = mode;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
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
        const p = new commonACFly.ExtDroneVersionDetectModeSetCommand();
        p.l_l = l_min;
        p.l_h = l_max;
        p.a_l = a_min;
        p.a_h = a_max;
        p.b_l = b_min;
        p.b_h = b_max;
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Emergency stop - stop motors immediately
     */
    stop() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 0; // disarm
        p._param2 = 21196; // magic number for emergency stop
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Hover in place - stop current movement
     */
    hover() {
        const p = new commonACFly.ExtDroneMoveCommand();
        p.direction = 1; // any direction
        p.distance = 0; // zero distance
        p.speed = 0; // zero speed
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Flip forward
     */
    flipForward() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 1; // forward
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Flip backward
     */
    flipBack() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 2; // backward
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Flip left
     */
    flipLeft() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 3; // left
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * Flip right
     */
    flipRight() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 4; // right
        p._param7 = getNowTimestampMsUintFloat();
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

}
