import moment from "moment";
import {
    AirplaneOwl02Interface,
    AirplaneOwl02State,
    FlyModeAutoEnum,
    FlyModeEnum,
    FlyModeStableEnum
} from "./AirplaneOwl02Interface";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {common, MavLinkData, MavLinkPacket, minimal, uint8_t} from "node-mavlink";
import {PackAndDataType} from "./CustomProtocolTransformManager";
import {TimeBasedFifoCache} from "./utils/TimeBasedFifoCache";
import {BehaviorSubject, filter, firstValueFrom, Subject} from "rxjs";
import {timeoutWithoutError} from "./utils/rxjsTimeoutWithoutError";
import * as commonACFly from './Owl02Lib/commonACFly';

export interface MavLinkPacketRecord<D extends MavLinkData = MavLinkData> {
    time: moment.Moment;
    pack: MavLinkPacket;
    msgId: uint8_t;
    data?: D;
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
            [common.GlobalPositionInt.MSG_ID]: this.parseGpsPos.bind(this),
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

    protected cachePacketRecord(pack: MavLinkPacket, data?: MavLinkData): void {
        const msgId = pack.header.msgid;
        const record: MavLinkPacketRecord = {
            time: moment(),
            pack: pack,
            msgId: msgId,
            data: data,
        } satisfies MavLinkPacketRecord;
        this.cachedPacketRecord.set(msgId, record);
        this.packStream.next(record);
    }

    public sendMsg(msg: MavLinkData) {
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    public async sendHeartbeat() {
        const commandHeartbeat = new minimal.Heartbeat();
        // commandHeartbeat.type = minimal.MavType.GCS;
        // commandHeartbeat.autopilot = minimal.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        // commandHeartbeat.baseMode = minimal.MavModeFlag.CUSTOM_MODE_ENABLED;
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
        // commandHeartbeat.customMode = 0;
        // commandHeartbeat.systemStatus = minimal.MavState.ACTIVE;
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
        const p = data.data as minimal.Heartbeat;
        this.state.isArmed = (p.baseMode & 0x80) === 0x80;
        // this.state.flyMode = ListSwitch((p.customMode >>> (8 * 3)) & 0xFF, {
        //     2: FlyModeEnum.FLY_MODE_HOLD,
        //     3: FlyModeEnum.FLY_MODE_POSITION,
        //     4: FlyModeEnum.FLY_MODE_AUTO,
        // }, FlyModeEnum.INVALID);
        this.state.flyMode = {
            2: FlyModeEnum.FLY_MODE_HOLD,
            3: FlyModeEnum.FLY_MODE_POSITION,
            4: FlyModeEnum.FLY_MODE_AUTO,
        }[(p.customMode >>> (8 * 3)) & 0xFF] ?? FlyModeEnum.INVALID;
        switch (this.state.flyMode) {
            case FlyModeEnum.FLY_MODE_AUTO:
                this.state.flyModeAuto = ListSwitch((p.customMode >>> (8 * 4)) & 0xFF, {
                    2: FlyModeAutoEnum.FLY_MODE_AUTO_TAKEOFF,
                    3: FlyModeAutoEnum.FLY_MODE_AUTO_FOLLOW,
                    4: FlyModeAutoEnum.FLY_MODE_AUTO_MISSION,
                    5: FlyModeAutoEnum.FLY_MODE_AUTO_RTL,
                    6: FlyModeAutoEnum.FLY_MODE_AUTO_LAND,
                }, FlyModeAutoEnum.INVALID);
                this.state.flyModeStable = FlyModeStableEnum.INVALID;
                break;
            case FlyModeEnum.FLY_MODE_POSITION:
                this.state.flyModeStable = ListSwitch((p.customMode >>> (8 * 4)) & 0xFF, {
                    0: FlyModeStableEnum.FLY_MODE_STABLE_NORMAL,
                    2: FlyModeStableEnum.FLY_MODE_STABLE_OBSTACLE_AVOIDANCE,
                }, FlyModeStableEnum.INVALID);
                this.state.flyModeAuto = FlyModeAutoEnum.INVALID;
            case FlyModeEnum.INVALID:
            default:
                this.state.flyModeAuto = FlyModeAutoEnum.INVALID;
                this.state.flyModeStable = FlyModeStableEnum.INVALID;
        }
    }

    protected parseLandState(data: PackAndDataType) {
        const p = data.data as common.ExtendedSysState;
        this.state.isLanded = p.landedState;
    }

    protected parseStatusText(data: PackAndDataType) {
        const p = data.data as common.StatusText;
        this.cacheStateText.push({
            time: moment(),
            pack: data.packet,
            msgId: data.packet.header.msgid,
            data: p,
        });
    }

    protected parseAutopilotVersion(data: PackAndDataType) {
        const p = data.data as common.AutopilotVersion;
        this.state.flightSwVersion = p.flightSwVersion;
        this.state.flightSwVersionString = [
            ((p.flightSwVersion >>> (8 * 2)) & 0xFF),
            ((p.flightSwVersion >>> (8 * 1)) & 0xFF),
            ((p.flightSwVersion >>> (8 * 0)) & 0xFF),
        ].join('.');
        this.state.boardVersion = p.boardVersion;
        // 序列号计算如下:
        // uint32_t UID0 = ((uint32_t*) uid2)[0];
        // uint32_t UID 1 = ((uint32_t*) uid2)[1];
        // uint32_t UID 2 = ((uint32_t*) uid2)[2];
        // 再将UID0- UID2转成16进制, 拼接起来即可得到完整的序列号
        this.state.SN = ((p.uid2[0] & 0xFFFFFFFF) >>> 0).toString(16).padStart(8, '0')
            + ((p.uid2[1] & 0xFFFFFFFF) >>> 0).toString(16).padStart(8, '0')
            + ((p.uid2[2] & 0xFFFFFFFF) >>> 0).toString(16).padStart(8, '0');
    }

    protected parseAck(data: PackAndDataType) {
        this.ackPackStream.next(data as PackAndDataType<common.CommandAck>);
        const p = data.data as common.CommandAck;
        // TODO
        console.log('[AirplaneOwl02] CommandAck:', p);
        const cmdId = p.command;
        const isOk = p.result === 0;
        const progress = p.progress;
    }

    protected parseGpsPos(data: PackAndDataType) {
        const p = data.data as common.GlobalPositionInt;
        this.state.gpsPosition.lat = p.lat / 1e7;
        this.state.gpsPosition.lon = p.lon / 1e7;
        this.state.gpsPosition.alt = p.alt / 1e4;
        this.state.gpsPosition.relativeAlt = p.relativeAlt / 1e4;
        this.state.gpsPosition.hdg = p.hdg;
    }

    protected parseBatteryStatusAcfly(data: PackAndDataType) {
        const p = data.data as commonACFly.BatteryStatusAcfly;
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
                text: T.data?.text ?? '',
                time: T.time,
                severity: T.data?.severity ?? 0,
            };
        });
    }

    public waitAckOnce(timeoutMs: number, command: common.MavCmd) {
        return firstValueFrom(this.ackPackStream.pipe(
            filter(T => !!T),
            filter(P => P.data.command === command),
            timeoutWithoutError(timeoutMs, undefined),
        ));
    }

}

export class AirplaneOwl02Commander {

    constructor(
        public airplane: AirplaneOwl02,
    ) {
    }

    lock() {
        const p = new common.ComponentArmDisarmCommand();
        p.arm = 0;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    unlock() {
        const p = new common.ComponentArmDisarmCommand();
        p.arm = 1;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * @param height m
     */
    takeoff(height: number) {
        const p = new common.CommandLong();
        p._param7 = height;
        p.command = common.MavCmd.NAV_TAKEOFF_LOCAL;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * 一键降落
     * @param landHeight    降落高度(相对起飞点)，单位m
     * @param yawAngle      飞机航向，单位度
     */
    land(landHeight: number, yawAngle: number) {
        const p = new common.NavLandCommand();
        p._param4 = yawAngle;
        p._param5 = 1000;
        p._param6 = 1000;
        p._param7 = landHeight;
        p.command = common.MavCmd.NAV_LAND;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    // 一键返航
    rtl() {
        const p = new common.NavLandCommand();
        p.command = common.MavCmd.NAV_RETURN_TO_LAUNCH;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * 一键控制载荷
     * @param pwdChn    PWM通道;    [1，14]
     * @param pwmRate    PWM占空比;  [0，65000]，50HZ
     */
    control_payload(pwdChn: number, pwmRate: number) {
        const p = new common.NavLandCommand();
        p.command = common.MavCmd.DO_SET_SERVO;
        p._param1 = pwdChn;
        p._param2 = pwmRate;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * gps定点飞行
     * @param lat       （纬度，单位°）
     * @param lon       （经度，单位°）
     * @param alt       （海拔高度，单位米）
     * @param yawAngle  （航向角，-360 ~360，填nan则指向航点再飞过去， 填1000则不旋转偏航）
     * @param speed     （速度，m/s）
     */
    gotoGps(lat: number, lon: number, alt: number, yawAngle: number, speed: number) {
        const p = new common.CommandInt();
        p._param1 = speed;
        p._param2 = common.MavDoRepositionFlags.CHANGE_MODE;
        p._param4 = yawAngle;
        p._param5 = lat;
        p._param6 = lon;
        p._param7 = alt;
        p.command = common.MavCmd.DO_REPOSITION;
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
    gotoLocal(x: number, y: number, h: number, yawAngle: number, speed: number) {
        const p = new common.CommandLong();
        p._param1 = speed;
        p._param2 = commonACFly.MavFrame.BODY_FLU;
        p._param4 = yawAngle;
        p._param5 = x;
        p._param6 = y;
        p._param7 = h;
        p.command = common.MavCmd.DO_REPOSITION;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    /**
     * @param mode
     * @param subMode
     */
    setFlyMode(mode: FlyModeEnum, subMode: FlyModeAutoEnum | FlyModeStableEnum) {
        const p = new common.NavLandCommand();
        p._param1 = 1;
        switch (mode) {
            case FlyModeEnum.FLY_MODE_HOLD:
                p._param2 = 2;
                break;
            case FlyModeEnum.FLY_MODE_POSITION:
                p._param2 = 3;
                switch (subMode) {
                    case FlyModeStableEnum.FLY_MODE_STABLE_NORMAL:
                        p._param3 = 0;
                        break;
                    case FlyModeStableEnum.FLY_MODE_STABLE_OBSTACLE_AVOIDANCE:
                        p._param3 = 2;
                        break;
                    case FlyModeStableEnum.INVALID:
                    default:
                        console.warn('[AirplaneOwl02Commander] setFlyMode invalid subMode for position mode', subMode);
                        return Promise.reject(new Error('[AirplaneOwl02Commander] setFlyMode invalid subMode for position mode ' + subMode));
                }
                break;
            case FlyModeEnum.FLY_MODE_AUTO:
                p._param2 = 4;
                switch (subMode) {
                    case FlyModeAutoEnum.FLY_MODE_AUTO_FOLLOW:
                        p._param3 = 3;
                        break;
                    case FlyModeAutoEnum.FLY_MODE_AUTO_MISSION:
                        p._param3 = 4;
                        break;
                    case FlyModeAutoEnum.FLY_MODE_AUTO_RTL:
                        p._param3 = 5;
                        break;
                    case FlyModeAutoEnum.FLY_MODE_AUTO_LAND:
                        p._param3 = 6;
                        break;
                    case FlyModeAutoEnum.FLY_MODE_AUTO_TAKEOFF:
                    // (不可设置，只反馈)
                    // p._param3 = 2;
                    // break;
                    case FlyModeAutoEnum.INVALID:
                    default:
                        console.warn('[AirplaneOwl02Commander] setFlyMode invalid subMode for auto mode', subMode);
                        return Promise.reject(new Error('[AirplaneOwl02Commander] setFlyMode invalid subMode for auto mode ' + subMode));
                }
                break;
            case FlyModeEnum.FLY_MODE_OFF_BOARD:
            case FlyModeEnum.INVALID:
            default:
                console.warn('[AirplaneOwl02Commander] setFlyMode invalid mode', mode);
                return Promise.reject(new Error('[AirplaneOwl02Commander] setFlyMode invalid mode ' + mode));
        }
        p.command = common.MavCmd.DO_SET_MODE;
        p.targetSystem = 1;
        p.targetComponent = 1;
        return this.airplane.sendMsg(p);
    }

    set_home_position(lat: number, lon: number, alt: number) {
        const p = new common.SetHomePosition();
        p.latitude = lat;
        p.longitude = lon;
        p.altitude = alt;
        p.targetSystem = 1;
        return this.airplane.sendMsg(p);
    }

}
