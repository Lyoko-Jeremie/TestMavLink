import moment from 'moment';
import {MavLinkData, MavLinkPacket, uint8_t} from "node-mavlink";
import {CustomProtocolTransformManager, PackAndDataType} from "./CustomProtocolTransformManager";

export interface MavLinkPacketRecord {
    time: moment.Moment;
    pack: MavLinkPacket;
    msgId: uint8_t;
    data?: MavLinkData;
}

export class MavStateItem {
    cachedPacketRecord: Map<uint8_t, MavLinkPacketRecord> = new Map<uint8_t, MavLinkPacketRecord>();

    // TODO collect state info from Record

    constructor(
        public deviceId: number,
    ) {
    }

    addPacketRecord(pack: MavLinkPacket, data?: MavLinkData): void {
        const msgId = pack.header.msgid;
        const record: MavLinkPacketRecord = {
            time: moment(),
            pack: pack,
            msgId: msgId,
            data: data,
        } satisfies MavLinkPacketRecord;
        this.cachedPacketRecord.set(msgId, record);

    }

    getStateInfo() {
        if (this.cachedPacketRecord.size === 0) {
            return undefined;
        }
        // TODO collect from Record to create info
        return {
            // TODO
        };
    }


}

export class MavStateCollector {
    protected mavStateTable: Map<number, MavStateItem> = new Map<number, MavStateItem>();

    constructor(
        public m: CustomProtocolTransformManager,
    ) {
        m.getMavLinkAllPackAndDataObservable().subscribe({
            next: (data) => {
                this.addPacketRecord(data.id, data.packAndData);
            },
        });
    }

    addPacketRecord(id: number, packAndData: PackAndDataType): void {
        let mavStateItem = this.mavStateTable.get(id);
        if (!mavStateItem) {
            mavStateItem = new MavStateItem(id);
            this.mavStateTable.set(id, mavStateItem);
        }
        mavStateItem.addPacketRecord(packAndData.packet, packAndData.data);
    }

    getStateInfo(deviceId: number) {
        return this.mavStateTable.get(deviceId)?.getStateInfo();
    }

}
