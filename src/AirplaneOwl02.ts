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

export interface MavLinkPacketRecord {
    time: moment.Moment;
    pack: MavLinkPacket;
    msgId: uint8_t;
    data?: MavLinkData;
}

function ListSwitch<I extends number | string, R>(o: I, s: Record<I, R>, d: R): R {
    // if (o in s) {
    //     return s[o];
    // }
    // return d;
    return s[o] ?? d;
}

export class AirplaneOwl02 implements AirplaneOwl02Interface {
    state: AirplaneOwl02State = new AirplaneOwl02State();
    cachedPacketRecord: Map<uint8_t, MavLinkPacketRecord> = new Map<uint8_t, MavLinkPacketRecord>();

    parseTable: { [key: number]: (data: PackAndDataType) => void };
    cachedPacketIds: Set<number>;

    cacheStateText: ;

    constructor(
        public targetChannelId: number,
        public manager: AirplaneManagerOwl02Interface,
    ) {
        this.parseTable = {
            [minimal.Heartbeat.MSG_ID]: this.parseHeartbeat.bind(this),
            [common.ExtendedSysState.MSG_ID]: this.parseLandState.bind(this),
            [common.AutopilotVersion.MSG_ID]: this.parseAutopilotVersion.bind(this),
            [common.StatusText.MSG_ID]: this.parseStatusText.bind(this),
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
        ]);
    }

    isInit = false;

    async init() {
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
    }

    protected sendMsg(msg: MavLinkData) {
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    async sendHeartbeat() {
        const commandHeartbeat = new minimal.Heartbeat();
        commandHeartbeat.type = minimal.MavType.GCS;
        commandHeartbeat.autopilot = minimal.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        commandHeartbeat.baseMode = minimal.MavModeFlag.CUSTOM_MODE_ENABLED;
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
        commandHeartbeat.customMode = 0;
        commandHeartbeat.systemStatus = minimal.MavState.ACTIVE;
        await this.sendMsg(commandHeartbeat);
    }

    /**
     * call this , and then the autopilot will send back AutopilotVersion pack
     */
    public async triggerGetAutopilotVersion() {
        const p = new common.RequestAutopilotCapabilitiesCommand();
        p.targetComponent = 1;
        p.targetComponent = 1;
        await this.sendMsg(p);
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
        p.text;
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
}
