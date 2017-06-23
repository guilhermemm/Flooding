//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "Flooding.h"

Define_Module(Flooding);

const simsignalwrap_t Flooding::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void Flooding::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {
        traci = Veins::TraCIMobilityAccess().get(getParentModule());

        //TODO: Added for Game Theory Solution
        lastTxPower = registerSignal("lastTxPower");
        meanSNR = registerSignal("meanSNR");

        sentDownMACInCCH = registerSignal("sentDownMACInCCH");
        collisions = registerSignal("collisions");
        duplicatedMessages = registerSignal("duplicatedMessages");
        messagesTransmitted = registerSignal("messagesTransmitted");
        messagesReceived = registerSignal("messagesReceived");
        isInROI = registerSignal("isInROI");

        wasInROI = false;
        disseminationStarted = false;

        getSimulation()->getSystemModule()->subscribe("disseminationStartTime", this);

        mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        assert(mac);

        //TODO: Added for Game Theory Solution
        curTxPower = mac->par("txPower");
        powerLevel = 3;
        myUtilityValue = 0;

        lastNumCollisions = 0;
        totalCollisions = 0;

        CCHStartTimer = new cMessage("CCH start", CCH_START);
        SCHStartTimer = new cMessage("SCH start", SCH_START);

        uint64_t currenTime = simTime().raw();
        uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
        double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();

        // Control Channel is active
        if ((currenTime / switchingTime) % 2 == 0) {
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch, SCHStartTimer);
        }
        // Service Channel is active
        else {
            scheduleAt(simTime() + timeToNextSwitch, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, SCHStartTimer);
        }
    }
}

void Flooding::finish() {
    BaseWaveApplLayer::finish();

    if (wasInROI) {
        emit(isInROI, 1);
    } else {
        emit(isInROI, 0);
    }

    //TODO: Added for Game Theory Solution
    emit(lastTxPower, curTxPower);

    emit(collisions, totalCollisions);

    if (!messagesRcvd.empty()) {
        std::ofstream log;
        std::ostringstream o;

        o << "./results/" << par("log_traffic").longValue() << "-" << par("log_replication").longValue() << "-receiver-" << myId;
        log.open(o.str().c_str());

        for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
            MessageInfoEntry* videoInfo = i->second;

            log << videoInfo->receptionTime << " " << "id " << videoInfo->messageID << " " << "udp " << videoInfo->messageLength << " " << videoInfo->distanceToOrigin << endl;
        }
        log.close();
    }
    getSimulation()->getSystemModule()->unsubscribe("disseminationStartTime", this);
}

void Flooding::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);

            Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
            if (simTime() > par("startDataProductionTime").doubleValue() - 3 &&
                    curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {

                sendWSM(wsm);
            } else {
                delete wsm;
            }

            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);

            break;
        }

        case BACK_TRAFFIC_ENTRY_TIMEOUT: {
            NeighborEntry* entry = (NeighborEntry*) msg->getContextPointer();
            int address = entry->senderAddress;
            cancelAndDelete(entry->beaconHoldTimer);
            entry->beaconHoldTimer = NULL;
            lastRequesters.erase(address);

            break;
        }

        case CCH_START: {
            totalCollisions = totalCollisions + mac->statsTXRXLostPackets - lastNumCollisions;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, CCHStartTimer);
            break;
        }

        case SCH_START: {
            lastNumCollisions = mac->statsTXRXLostPackets;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, SCHStartTimer);
            break;
        }

        default: {
            if (msg)
                EV << "Flooding - Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

void Flooding::onBeacon(WaveShortMessage* wsm) {
    Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
    // if back-traffic is enabled, then generate it only three seconds before the main dissemination.
    if (par("generateBackTraffic").boolValue() && simTime() > par("startDataProductionTime").doubleValue() - 3
            && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {
        processBackTraffic(wsm->getSenderAddress());
    }
}

void Flooding::onData(WaveShortMessage* wsm) {
    //TODO: Added for GAme Theory Solution
    if (par("adaptTxPower").boolValue())
        adjustTxPower(wsm);

    MessageInfoEntry* info = extractMsgInfo(wsm);

    if (!isInsideROI(info) || !isMessageAlive(info)) {
        return;
    }

    if (!isDuplicateMsg(info->messageID)) {
        // Store message info
        messagesRcvd[info->messageID] = info;
        emit(messagesReceived, 1);

        WaveShortMessage* wsm = createDataMsg(info);
        sendWSM(wsm);

        emit(messagesTransmitted, 1);

        if (isCCHActive()) {
            emit(sentDownMACInCCH, 1);
        }
    } else {
        emit(duplicatedMessages, 1);
    }
}

bool Flooding::isCCHActive() {
    uint64_t currenTime = simTime().raw();
    uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();

    return ((currenTime / switchingTime) % 2 == 0);
}

//TODO: Added for Game Theory Solution
void Flooding::adjustTxPower(WaveShortMessage* wsm) {
    double rcvTxPower, rcvSNR, utility;

    rcvTxPower = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    // Convert dBm to mW
    rcvTxPower = pow(10.0, rcvTxPower / 10.0);
    rcvSNR = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr();

    utility = (1 * log(1 + rcvSNR) - (0.009 * rcvTxPower));

    //if (rcvTxPower <= curTxPower) {
        if (myUtilityValue < utility) {
            myUtilityValue = utility;
            //increaseTxPower();
            decreaseTxPower();
        } else {
            myUtilityValue = utility;
            //decreaseTxPower();
            increaseTxPower();
        }
    //}

    emit(meanSNR, rcvSNR);
}

//TODO: Added for Game Theory Solution
void Flooding::decreaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel > 0) {
        powerLevel--;
        curTxPower = txPowerVals[powerLevel];
    }
}

//TODO: Added for Game Theory Solution
void Flooding::increaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel < 3) {
        powerLevel++;
        curTxPower = txPowerVals[powerLevel];
    }

}

Flooding::MessageInfoEntry* Flooding::extractMsgInfo(WaveShortMessage* wsm) {
    if (isDuplicateMsg(wsm->getSerial())) {
        return messagesRcvd[wsm->getSerial()];
    }
    DataMessage* dataMsg = dynamic_cast<DataMessage*>(wsm->decapsulate());

    MessageInfoEntry* info = new MessageInfoEntry;

    info->messageID = wsm->getSerial();
    info->messageOriginPosition = dataMsg->getMessageOriginPosition();
    info->messageROI = dataMsg->getMessageROI();
    info->messageOriginTime = dataMsg->getMessageOriginTime();
    info->messageTTL = dataMsg->getMessageTTL();
    info->hops = dataMsg->getHops() + 1;
    info->receptionTime = simTime();
    info->messageLength = wsm->getByteLength();
    info->distanceToOrigin = info->messageOriginPosition.distance(curPosition);

    delete dataMsg;

    return info;
}

bool Flooding::isDuplicateMsg(int messageID) {
    return messagesRcvd.find(messageID) != messagesRcvd.end();
}

bool Flooding::isInsideROI(MessageInfoEntry* info) {
    return info->messageOriginPosition.distance(curPosition) < info->messageROI;
}

bool Flooding::isMessageAlive(MessageInfoEntry* info) {
    return simTime() < info->messageOriginTime + info->messageTTL;
}

WaveShortMessage* Flooding::createDataMsg(MessageInfoEntry* info) {
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, type_SCH, dataPriority, 0, info->messageID);

    //TODO: Added for Game Theory Solution
    PhyControlMessage *controlInfo = new PhyControlMessage();
    controlInfo->setTxPower_mW(curTxPower);
    wsm->setControlInfo(dynamic_cast<cObject *>(controlInfo));

    DataMessage* msg = new DataMessage("data");

    msg->setMessageOriginPosition(info->messageOriginPosition);
    msg->setMessageROI(info->messageROI);
    msg->setMessageOriginTime(info->messageOriginTime);
    msg->setMessageTTL(info->messageTTL);
    msg->setHops(info->hops);

    wsm->setByteLength(info->messageLength);

    wsm->encapsulate(msg);

    return wsm;
}

void Flooding::processBackTraffic(int senderAddr) {
    if (lastRequesters.find(senderAddr) == lastRequesters.end()) {
        // Send 60 packets of 1000 bytes on the Service Channel
        for (int i = 0; i < 60; i++) {
            WaveShortMessage* wsm = prepareWSM("back traffic", dataLengthBits, type_SCH, dataPriority, 0, i);
            wsm->setByteLength(1000);

            sendWSM(wsm);
        }

        NeighborEntry* entry = new NeighborEntry;

        entry->beaconHoldTimer = new cMessage("beacon entry timeout", BACK_TRAFFIC_ENTRY_TIMEOUT);
        entry->beaconHoldTimer->setContextPointer((NeighborEntry*) entry);
        scheduleAt(simTime() + 3, entry->beaconHoldTimer);

        lastRequesters[senderAddr] = entry;
    }
}

void Flooding::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    BaseWaveApplLayer::receiveSignal(source, signalID, obj, details);

    if (signalID == mobilityStateChangedSignal) {
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (disseminationStarted && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() &&
                ((simTime() >= disseminationStartTime) && (simTime() <= (disseminationStartTime + par("dataTTL").doubleValue())))) {
            wasInROI = true;
        }
    }
}

void Flooding::receiveSignal(cComponent *source, simsignal_t signalID, const SimTime& t) {
    Enter_Method_Silent();

    if (!strcmp(getSignalName(signalID), "disseminationStartTime")) {
        disseminationStarted = true;
        disseminationStartTime = simTime();
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (curPosition.distance(rsuPosition) <= par("dataROI").doubleValue())
            wasInROI = true;
    }
}

Flooding::~Flooding() {
    for (std::map<int, NeighborEntry*>::iterator i = lastRequesters.begin(); i != lastRequesters.end(); i++) {
        NeighborEntry* entry = i->second;
        cancelAndDelete(entry->beaconHoldTimer);
    }
    cancelAndDelete(CCHStartTimer);
    cancelAndDelete(SCHStartTimer);
}
