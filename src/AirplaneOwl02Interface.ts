import {common} from "node-mavlink";

export interface AirplaneOwl02Interface {
    targetChannelId: number;
}

export class AirplaneOwl02State {
    isArmed: boolean = false;

    isLanded: common.MavLandedState = common.MavLandedState.UNDEFINED;

}
