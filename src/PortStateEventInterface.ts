import type {Subject} from "rxjs";

export interface PortStateEventInterface {
    portCloseEvent: Subject<void>;
    portIsOpen(): boolean;
}
