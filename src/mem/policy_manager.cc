/*
 * Copyright (c) 2023 The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/policy_manager.hh"

#include "base/trace.hh"
#include "debug/PolicyManager.hh"
#include "debug/Drain.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

bool meetsPrintCriteria(PacketPtr pkt) {
    size_t pkt_size = pkt->getSize();
    uint8_t * ptr = pkt->getPtr<uint8_t>();
    int counter = 0;

    // uint8_t initial_data = *ptr;
    // ptr++;
    // if (initial_data >= 'A' && initial_data <= 'Z') {
    //     for (unsigned int i = 1; i < pkt_size; i++) {
    //         uint8_t data = *ptr;
    //         if (data == initial_data) {
    //             counter++;
    //         }
    //         ptr++;
    //     }
    // }

    for (unsigned int i = 0; i < pkt_size; i++) {
        uint8_t data = *ptr;
        if (data >= 'E' && data <= 'H') {
            counter++;
        }
        ptr++;
    }

    return counter > 62;
}

void printPacketAsChar(PacketPtr pkt) {
    size_t pkt_size = pkt->getSize();
    uint8_t * ptr = pkt->getPtr<uint8_t>();

    printf("packet: ");
    for (unsigned int i = 0; i < pkt_size; i++) {
        uint8_t data = *ptr;
        printf("%c", data);
        ptr++;
    }
    printf("\n");
}

PolicyManager::PolicyManager(const PolicyManagerParams &p):
    AbstractMemory(p),
    port(name() + ".port", *this),
    locReqPort(name() + ".loc_req_port", *this),
    farReqPort(name() + ".far_req_port", *this),
    locBurstSize(p.loc_burst_size),
    farBurstSize(p.far_burst_size),
    locMemPolicy(p.loc_mem_policy),
    dramCacheSize(p.dram_cache_size),
    blockSize(p.block_size),
    addrSize(p.addr_size),
    orbMaxSize(p.orb_max_size), orbSize(0),
    crbMaxSize(p.crb_max_size), crbSize(0),
    alwaysHit(p.always_hit), alwaysDirty(p.always_dirty),
    ltt_table_size(p.ltt_table_size),
    inst_based_MAP_table_size(p.inst_based_MAP_table_size),
    inst_based_MAP_bit_vector_size(p.inst_based_MAP_bit_vector_size),
    bypassDcache(p.bypass_dcache),
    frontendLatency(p.static_frontend_latency),
    backendLatency(p.static_backend_latency),
    tRP(p.tRP),
    tRCD_RD(p.tRCD_RD),
    tRL(p.tRL),
    numColdMisses(0),
    cacheWarmupRatio(p.cache_warmup_ratio),
    fpc_compressor(p.fpc_compressor),
    bdi_compressor(p.bdi_compressor),
    resetStatsWarmup(false),
    retryLLC(false), retryLLCFarMemWr(false),
    retryLocMemRead(false), retryFarMemRead(false),
    retryLocMemWrite(false), retryFarMemWrite(false),
    maxConf(0),
    locMemReadEvent([this]{ processLocMemReadEvent(); }, name()),
    locMemWriteEvent([this]{ processLocMemWriteEvent(); }, name()),
    farMemReadEvent([this]{ processFarMemReadEvent(); }, name()),
    farMemWriteEvent([this]{ processFarMemWriteEvent(); }, name()),
    polManStats(*this)
{
    panic_if(orbMaxSize<8, "ORB maximum size must be at least 8.\n");

    tagMetadataStore.resize(dramCacheSize/blockSize);
    tagBaiMetadataStore.resize(dramCacheSize/blockSize);
    for(int i = 0; i < dramCacheSize/blockSize; i++) tagBaiMetadataStore[i].resize(2);

    //Init last time table to false
    for(int i = 0; i<ltt_table_size; i++) last_time_table[i] = false;

    //Init instruction based MAP table to false
    for(int i = 0; i<inst_based_MAP_table_size; i++) inst_based_MAP_table[i] = 0;

    // Init compressors for DICE
    // fpc_compressor = 
    // bdi_compressor = 

    if (fpc_compressor != nullptr) {
        printf("FPC compressor block size: %lu\n", fpc_compressor->getBlockSize());
    }
    if (bdi_compressor != nullptr) {
        printf("BDI compressor block size: %lu\n", bdi_compressor->getBlockSize());
    }
}

Tick
PolicyManager::recvAtomic(PacketPtr pkt)
{
    if (!getAddrRange().contains(pkt->getAddr())) {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }

    DPRINTF(PolicyManager, "recvAtomic: %s 0x%x\n",
                     pkt->cmdString(), pkt->getAddr());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    // do the actual memory access and turn the packet into a response
    access(pkt);

    if (pkt->hasData()) {
        // this value is not supposed to be accurate, just enough to
        // keep things going, mimic a closed page
        // also this latency can't be 0
        // panic("Can't handle this process --> implement accessLatency() "
        //         "according to your interface. pkt: %s\n", pkt->print());
        return accessLatency();
    }

    return 0;
}

Tick
PolicyManager::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    Tick latency = recvAtomic(pkt);
    getBackdoor(backdoor);
    return latency;
}

size_t hash_to_table(const std::string& key, size_t table_size) {
    std::hash<std::string> hash_fn; // Use std::hash for strings, you can change it for other types

    // Apply hash function and take modulo to fit into the table size
    return hash_fn(key) % table_size;
}

void
PolicyManager::recvFunctional(PacketPtr pkt)
{
    bool found;

    if (getAddrRange().contains(pkt->getAddr())) {
        // rely on the abstract memory
        functionalAccess(pkt);
        found = true;
    } else {
        found = false;
    }

    panic_if(!found, "Can't handle address range for packet %s\n",
             pkt->print());
}

Tick
PolicyManager::accessLatency()
{
    // THIS IS FOR DRAM ONLY!
    return (tRP + tRCD_RD + tRL);
}

void
PolicyManager::init()
{
   if (!port.isConnected()) {
        fatal("Policy Manager %s is unconnected!\n", name());
    } else if (!locReqPort.isConnected()) {
        fatal("Policy Manager %s is unconnected!\n", name());
    } else if (!farReqPort.isConnected()) {
        fatal("Policy Manager %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
        //reqPort.recvRangeChange();
    }
}

bool
PolicyManager::recvTimingReq(PacketPtr pkt)
{
    
    DPRINTF(PolicyManager, "recvTimingReq: frontendLatency = %d backendLatency = %d\n",frontendLatency, backendLatency);

    //For writes, always go to DRAM cache
    if(pkt->isRead()){

        pkt->bypass_dcache = read_MAPI(pkt->getAddr());

        bool NormHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnIndexDC(pkt->getAddr(), pkt->getSize()), 0);
        bool BaiHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnBAIDC(pkt->getAddr(), pkt->getSize()), 1);
        
        if(NormHit || BaiHit)
            pkt->bypass_dcache = false;

    }

    if (pkt->bypass_dcache) {
        DPRINTF(PolicyManager, "Sending Req to memory\n");
        return farReqPort.sendTimingReq(pkt);
    }
    // This is where we enter from the outside world
    DPRINTF(PolicyManager, "recvTimingReq: request %s addr 0x%x size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller\n");
    assert(pkt->getSize() != 0);

    // Calc avg gap between requests
    if (prevArrival != 0) {
        polManStats.totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();

    // Find out how many memory packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one memory packet. Otherwise, a pkt translates to
    // multiple memory packets

    const Addr base_addr = pkt->getAddr();
    Addr addr = base_addr;
    uint32_t burst_size = locBurstSize;
    unsigned size = std::min((addr | (burst_size - 1)) + 1,
                    base_addr + pkt->getSize()) - addr;

    // check merging for writes
    if (pkt->isWrite()) {

        // polManStats.writePktSize[ceilLog2(size)]++;
 
        bool merged = isInWriteQueue.find((addr & ~(Addr(locBurstSize - 1)))) !=
            isInWriteQueue.end();

        if (merged) {

            polManStats.mergedWrBursts++;

            // farMemCtrl->accessInterface(pkt);

            // sendRespondToRequestor(pkt, frontendLatency);

            // return true;
        }
    }

    // check forwarding for reads
    bool foundInORB = false;
    bool foundInCRB = false;
    //bool foundInFarMemWrite = false;

    if (pkt->isRead()) {
        //pkt->predCompressible = (curTick()%128 == 0)? true : false;
        pkt->predCompressible = read_LTT(pkt->getAddr());
        pkt->compressed_index =  (pkt->getAddr()%(blockSize*2) / blockSize);

        access(pkt);//Experiment
        //DAN: We need the compressibility check here as well
        pkt->isCompressible = isCacheLineCompressible(pkt);
        
        //Updating LTT based on is_Compressible of read request
        if(pkt->isCompressible)
            update_LTT(pkt->getAddr(),true);
        else
            update_LTT(pkt->getAddr(),false);

        //Experimenting 
        DPRINTF(PolicyManager, "Read: request %s addr 0x%x | Prediction =  %d | Actual = %d \n",
            pkt->cmdString(), pkt->getAddr(), pkt->predCompressible, pkt->isCompressible);

        if (isInWriteQueue.find(pkt->getAddr()) != isInWriteQueue.end()) {

            if (!ORB.empty()) {
                for (const auto& e : ORB) {

                    // check if the read is subsumed in the write queue
                    // packet we are looking at
                    if (e.second->validEntry &&
                        e.second->owPkt->isWrite() &&
                        e.second->owPkt->getAddr() <= addr &&
                        ((addr + size) <=
                        (e.second->owPkt->getAddr() +
                        e.second->owPkt->getSize()))) {

                        foundInORB = true;

                        polManStats.servicedByWrQ++;

                        polManStats.bytesReadWrQ += burst_size;

                        break;
                    }
                }
            }

            if (!foundInORB && !CRB.empty()) {
                for (const auto& e : CRB) {

                    // check if the read is subsumed in the write queue
                    // packet we are looking at
                    if (e.second->isWrite() &&
                        e.second->getAddr() <= addr &&
                        ((addr + size) <=
                        (e.second->getAddr() + e.second->getSize()))) {

                        foundInCRB = true;

                        polManStats.servicedByWrQ++;

                        polManStats.bytesReadWrQ += burst_size;

                        break;
                    }
                }
            }

            if (!foundInORB && !foundInCRB && !pktFarMemWrite.empty()) {
                for (const auto& e : pktFarMemWrite) {
                    // check if the read is subsumed in the write queue
                    // packet we are looking at
                    if (e.second->getAddr() <= addr &&
                        ((addr + size) <=
                        (e.second->getAddr() +
                         e.second->getSize()))) {

                        // foundInFarMemWrite = true;

                        polManStats.servicedByWrQ++;

                        polManStats.bytesReadWrQ += burst_size;

                        break;
                    }
                }
            }
        }

        // if (foundInORB || foundInCRB || foundInFarMemWrite) {
        //     polManStats.readPktSize[ceilLog2(size)]++;

        //     farMemCtrl->accessInterface(pkt);

        //     sendRespondToRequestor(pkt, frontendLatency);

        //     return true;
        // }
    }

    // process conflicting requests.
    // conflicts are checked only based on Index of DRAM cache
    if (checkConflictInDramCache(pkt)) {

        polManStats.totNumConf++;

        if (CRB.size()>=crbMaxSize) {

            DPRINTF(PolicyManager, "CRBfull: %lld\n", pkt->getAddr());

            polManStats.totNumCRBFull++;

            retryLLC = true;

            if (pkt->isRead()) {
                polManStats.numRdRetry++;
            }
            else {
                polManStats.numWrRetry++;
            }
            return false;
        }

        CRB.push_back(std::make_pair(curTick(), pkt));

        if (pkt->isWrite()) {
            isInWriteQueue.insert(pkt->getAddr());
        }

        if (CRB.size() > maxConf) {
            maxConf = CRB.size();
            polManStats.maxNumConf = CRB.size();
        }
        return true;
    }
    // check if ORB or FMWB is full and set retry
    if (pktFarMemWrite.size() >= (orbMaxSize / 2)) {

        DPRINTF(PolicyManager, "FMWBfull: %lld\n", pkt->getAddr());

        retryLLCFarMemWr = true;

        if (pkt->isRead()) {
            polManStats.numRdRetry++;
        }
        else {
            polManStats.numWrRetry++;
        }
        return false;
    }

    if (ORB.size() >= orbMaxSize) {

        DPRINTF(PolicyManager, "ORBfull: addr %lld\n", pkt->getAddr());

        polManStats.totNumORBFull++;

        retryLLC = true;

        if (pkt->isRead()) {
            polManStats.numRdRetry++;
        }
        else {
            polManStats.numWrRetry++;
        }
        return false;
    }

    // if none of the above cases happens,
    // add ir to the ORB
    handleRequestorPkt(pkt);

    if (pkt->isWrite()) {
        isInWriteQueue.insert(pkt->getAddr());
    }

    // pktLocMemRead.push_back(pkt->getAddr());

    // polManStats.avgLocRdQLenEnq = pktLocMemRead.size();

    setNextState(ORB.at(pkt->getAddr()));

    handleNextState(ORB.at(pkt->getAddr()));

    DPRINTF(PolicyManager, "Policy manager accepted packet %lld\n", pkt->getAddr());

    return true;
}

void
PolicyManager::processLocMemReadEvent()
{
    // sanity check for the chosen packet
    auto orbEntry = ORB.at(pktLocMemRead.front());
    assert(orbEntry->validEntry);
    assert(orbEntry->state == locMemRead);
    assert(!orbEntry->issued);

    PacketPtr rdLocMemPkt = getPacket(pktLocMemRead.front(),
                                   blockSize,
                                   MemCmd::ReadReq);
    
    //NS:
    rdLocMemPkt->latencyFactor = orbEntry->owPkt->latencyFactor;
    /*
    uint8_t* data_recv = rdLocMemPkt->getPtr<uint8_t>();

    for(int i = 0; i < 8; i++) {
        DPRINTF(PolicyManager, "NS: processLocMemReadEvent DATA for idx %d is : %x\n", i, data_recv[i]);
    }
    */

    if (locReqPort.sendTimingReq(rdLocMemPkt)) {
        DPRINTF(PolicyManager, "loc mem read is sent : %lld\n", rdLocMemPkt->getAddr());
        orbEntry->state = waitingLocMemReadResp;
        orbEntry->issued = true;
        orbEntry->locRdIssued = curTick();
        pktLocMemRead.pop_front();
        polManStats.sentLocRdPort++;
    } else {
        DPRINTF(PolicyManager, "loc mem read sending failed: %lld\n", rdLocMemPkt->getAddr());
        retryLocMemRead = true;
        delete rdLocMemPkt;
        polManStats.failedLocRdPort++;
    }

    if (!pktLocMemRead.empty() && !locMemReadEvent.scheduled() && !retryLocMemRead) {
        schedule(locMemReadEvent, curTick()+1000);
    }
}

void
PolicyManager::processLocMemWriteEvent()
{
    // sanity check for the chosen packet
    auto orbEntry = ORB.at(pktLocMemWrite.front());
    assert(orbEntry->validEntry);
    assert(orbEntry->state == locMemWrite);
    assert(!orbEntry->issued);

    PacketPtr wrLocMemPkt = getPacket(pktLocMemWrite.front(),
                                   blockSize,
                                   MemCmd::WriteReq);

    if (locReqPort.sendTimingReq(wrLocMemPkt)) {

        DPRINTF(PolicyManager, "loc mem write is sent : %lld\n", wrLocMemPkt->getAddr());
        orbEntry->state = waitingLocMemWriteResp;
        orbEntry->issued = true;
        orbEntry->locWrIssued = curTick();
        pktLocMemWrite.pop_front();
        polManStats.sentLocWrPort++;
    } else {
        DPRINTF(PolicyManager, "loc mem write sending failed: %lld\n", wrLocMemPkt->getAddr());
        retryLocMemWrite = true;
        delete wrLocMemPkt;
        polManStats.failedLocWrPort++;
    }

    if (!pktLocMemWrite.empty() && !locMemWriteEvent.scheduled() && !retryLocMemWrite) {
        schedule(locMemWriteEvent, curTick()+1000);
    }
}

void
PolicyManager::processFarMemReadEvent()
{
    // sanity check for the chosen packet
    auto orbEntry = ORB.at(pktFarMemRead.front());
    assert(orbEntry->validEntry);
    assert(orbEntry->state == farMemRead);
    assert(!orbEntry->issued);

    PacketPtr rdFarMemPkt = getPacket(pktFarMemRead.front(),
                                      blockSize,
                                      MemCmd::ReadReq);

    if (farReqPort.sendTimingReq(rdFarMemPkt)) {
        DPRINTF(PolicyManager, "far mem read is sent : %lld\n", rdFarMemPkt->getAddr());
        orbEntry->state = waitingFarMemReadResp;
        orbEntry->issued = true;
        orbEntry->farRdIssued = curTick();
        pktFarMemRead.pop_front();
        polManStats.sentFarRdPort++;
    } else {
        DPRINTF(PolicyManager, "far mem read sending failed: %lld\n", rdFarMemPkt->getAddr());
        retryFarMemRead = true;
        delete rdFarMemPkt;
        polManStats.failedFarRdPort++;
    }

    if (!pktFarMemRead.empty() && !farMemReadEvent.scheduled() && !retryFarMemRead) {
        schedule(farMemReadEvent, curTick()+1000);
    }
}

void
PolicyManager::processFarMemWriteEvent()
{
    PacketPtr wrFarMemPkt = getPacket(pktFarMemWrite.front().second->getAddr(),
                                      blockSize,
                                      MemCmd::WriteReq);
    DPRINTF(PolicyManager, "FarMemWriteEvent: request %s addr %#x\n",
            wrFarMemPkt->cmdString(), wrFarMemPkt->getAddr());

    /*
    uint8_t* data_recv = wrFarMemPkt->getPtr<uint8_t>();

    for(int i = 0; i < 8; i++) {
        DPRINTF(PolicyManager, "NS: processFarMemWriteEvent DATA for idx %d is : %x\n", i, data_recv[i]);
    }
    */

    if (farReqPort.sendTimingReq(wrFarMemPkt)) {
        DPRINTF(PolicyManager, "far mem write is sent : %lld\n", wrFarMemPkt->getAddr());
        pktFarMemWrite.pop_front();
        polManStats.sentFarWrPort++;
    } else {
        DPRINTF(PolicyManager, "far mem write sending failed: %lld\n", wrFarMemPkt->getAddr());
        retryFarMemWrite = true;
        delete wrFarMemPkt;
        polManStats.failedFarWrPort++;
    }

    if (!pktFarMemWrite.empty() && !farMemWriteEvent.scheduled() && !retryFarMemWrite) {
        schedule(farMemWriteEvent, curTick()+1000);
    } else {
        if (drainState() == DrainState::Draining && pktFarMemWrite.empty() &&
            ORB.empty()) {
            DPRINTF(Drain, "PolicyManager done draining in farMemWrite\n");
            signalDrainDone();
        }
    }

    if (retryLLCFarMemWr && pktFarMemWrite.size()< (orbMaxSize / 2)) {

        DPRINTF(PolicyManager, "retryLLCFarMemWr sent\n");

        retryLLCFarMemWr = false;

        port.sendRetryReq();
    }
}

bool
PolicyManager::locMemRecvTimingResp(PacketPtr pkt)
{
    DPRINTF(PolicyManager, "locMemRecvTimingResp : %lld\n", pkt->getAddr());
    auto orbEntry = ORB.at(pkt->getAddr());

    /*
    uint8_t* data_recv = pkt->getPtr<uint8_t>();

    for(int i = 0; i < 8; i++) {
        DPRINTF(PolicyManager, "NS: locMemRecvTimingResp DATA for idx %d is : %x\n", i, data_recv[i]);
    }*/

    if (pkt->isRead()) {
        assert(orbEntry->state == waitingLocMemReadResp);

        if (orbEntry->handleDirtyLine &&
            (orbEntry->pol == enums::CascadeLakeNoPartWrs ||
            orbEntry->pol == enums::RambusHypo ||
            orbEntry->pol ==  enums::BearWriteOpt)
        ) {
            assert(!orbEntry->isHit);
            handleDirtyCacheLine(orbEntry);
        }
        orbEntry->locRdExit = curTick();
    }
    else {
        assert(pkt->isWrite());
        assert(orbEntry->state == waitingLocMemWriteResp);
        orbEntry->locWrExit = curTick();
    }

    // IMPORTANT:
    // orbEntry should not be used as the passed argument in setNextState and
    // handleNextState functions, reason: it's possible that orbEntry may be
    // deleted and updated, which will not be reflected here in the scope of
    // current lines since it's been read at line #475.
    setNextState(ORB.at(pkt->getAddr()));

    handleNextState(ORB.at(pkt->getAddr()));

    delete pkt;

    return true;
}

bool
PolicyManager::farMemRecvTimingResp(PacketPtr pkt)
{
    //pkt->bypass_dcache = read_MAPI(pkt->getAddr());
    //DPRINTF(PolicyManager, "read_MAPI : addr 0x%0x bypass_dcache %0d \n",pkt->getAddr(),pkt->bypass_dcache);
    if (pkt->bypass_dcache) {
        DPRINTF(PolicyManager, "Sending Resp back to source\n");
        access(pkt); //NS: Required since the actual data is supplied by the policyManager

        port.schedTimingResp(pkt, curTick());
        return true;
    }

    DPRINTF(PolicyManager, "farMemRecvTimingResp : %lld , %s \n", pkt->getAddr(), pkt->cmdString());

    if (pkt->isRead()) {

        auto orbEntry = ORB.at(pkt->getAddr());

        DPRINTF(PolicyManager, "farMemRecvTimingResp : continuing to far read resp: %d\n",
        orbEntry->owPkt->isRead());

        assert(orbEntry->state == waitingFarMemReadResp);

        orbEntry->farRdExit = curTick();

        // IMPORTANT:
        // orbEntry should not be used as the passed argument in setNextState and
        // handleNextState functions, reason: it's possible that orbEntry may be
        // deleted and updated, which will not be reflected here in the scope of
        // current lines since it's been read at line #508.
        setNextState(ORB.at(pkt->getAddr()));

        // The next line is absolutely required since the orbEntry will
        // be deleted and renewed within setNextState()
        // orbEntry = ORB.at(pkt->getAddr());

        handleNextState(ORB.at(pkt->getAddr()));

        delete pkt;
    }
    else {
        assert(pkt->isWrite());
        delete pkt;
    }

    return true;
}

void
PolicyManager::locMemRecvReqRetry()
{
    // assert(retryLocMemRead || retryLocMemWrite);
    bool schedRd = false;
    bool schedWr = false;
    if (retryLocMemRead) {

        if (!locMemReadEvent.scheduled() && !pktLocMemRead.empty()) {
            schedule(locMemReadEvent, curTick());
        }
        retryLocMemRead = false;
        schedRd = true;
    }
    if (retryLocMemWrite) {
        if (!locMemWriteEvent.scheduled() && !pktLocMemWrite.empty()) {
            schedule(locMemWriteEvent, curTick());
        }
        retryLocMemWrite = false;
        schedWr = true;
    }
    if (!schedRd && !schedWr) {
            // panic("Wrong local mem retry event happend.\n");

            // TODO: there are cases where none of retryLocMemRead and retryLocMemWrite
            // are true, yet locMemRecvReqRetry() is called. I should fix this later.
            if (!locMemReadEvent.scheduled() && !pktLocMemRead.empty()) {
                schedule(locMemReadEvent, curTick());
            }
            if (!locMemWriteEvent.scheduled() && !pktLocMemWrite.empty()) {
                schedule(locMemWriteEvent, curTick());
            }
    }

    DPRINTF(PolicyManager, "locMemRecvReqRetry: %d , %d \n", schedRd, schedWr);
}

void
PolicyManager::farMemRecvReqRetry()
{
    if (bypassDcache) {
        DPRINTF(PolicyManager, "Sending Retry Req to source");
        port.sendRetryReq();
        return;
    }

    assert(retryFarMemRead || retryFarMemWrite);

    bool schedRd = false;
    bool schedWr = false;

    if (retryFarMemRead) {
        if (!farMemReadEvent.scheduled() && !pktFarMemRead.empty()) {
            schedule(farMemReadEvent, curTick());
        }
        retryFarMemRead = false;
        schedRd = true;
    }
    if (retryFarMemWrite) {
        if (!farMemWriteEvent.scheduled() && !pktFarMemWrite.empty()) {
            schedule(farMemWriteEvent, curTick());
        }
        retryFarMemWrite = false;
        schedWr = true;
    }
    // else {
    //     panic("Wrong far mem retry event happend.\n");
    // }

    DPRINTF(PolicyManager, "farMemRecvReqRetry: %d , %d \n", schedRd, schedWr);
}

void
PolicyManager::setNextState(reqBufferEntry* orbEntry)
{
    orbEntry->issued = false;
    enums::Policy pol = orbEntry->pol;
    reqState state = orbEntry->state;
    bool isRead = orbEntry->owPkt->isRead();
    bool isHit = orbEntry->isHit;
    bool isDirty = checkDirty(orbEntry->owPkt->getAddr(), (/*orbEntry->owPkt->predCompressible || */orbEntry->owPkt->isCompressible)); //NS

    // start --> read tag
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->state == start) {
            orbEntry->state = locMemRead;
            orbEntry->locRdEntered = curTick();
            return;
    }

    // tag ready && read && hit --> DONE
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        orbEntry->isHit) {
            // done
            // do nothing
            return;
    }

    // tag ready && write --> loc write
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isWrite() &&
        orbEntry->state == waitingLocMemReadResp) {
            //NS: Checking if the compressibility check has been done or not
            DPRINTF(PolicyManager, "Setting locMemWrite State | isCompressible = %d\n", orbEntry->owPkt->isCompressible);

            // write it to the DRAM cache
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            return;
    }

    // loc read resp ready && read && miss --> far read
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        !orbEntry->isHit) {

            orbEntry->state = farMemRead;
            orbEntry->farRdEntered = curTick();
            return;
    }

    // far read resp ready && read && miss --> loc write
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingFarMemReadResp &&
        !orbEntry->isHit) {

            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency*5);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());

            // if (orbEntry->handleDirtyLine) {
            //     handleDirtyCacheLine(orbEntry);
            // }
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            return;
    }

    // loc write received
    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        // orbEntry->owPkt->isRead() &&
        // !orbEntry->isHit &&
        orbEntry->state == waitingLocMemWriteResp) {
            // done
            // do nothing
            return;
    }

    ////////////////////////////////////////////////////////////////////////
    /// RambusHypo

    // RD Hit Dirty & Clean, RD Miss Dirty, WR Miss Dirty
    // start --> read loc
    if (pol == enums::RambusHypo && state == start &&
        ((isRead && isHit) || (isRead && !isHit && isDirty) || (!isRead && !isHit && isDirty))
       ) {
            orbEntry->state = locMemRead;
            orbEntry->locRdEntered = curTick();
            return;
    }
    // RD Miss Clean
    // start --> read far
    if (pol == enums::RambusHypo && state == start &&
        (isRead && !isHit && !isDirty)
       ) {
            orbEntry->state = farMemRead;
            orbEntry->farRdEntered = curTick();
            return;
    }
    // WR Hit Dirty & Clean, WR Miss Clean
    // start --> write loc
    if (pol == enums::RambusHypo && state == start &&
        ((!isRead && isHit)|| (!isRead && !isHit && !isDirty))
       ) {
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            return;
    }

    // RD Hit Dirty & Clean
    // start --> read loc --> done
    if (pol == enums::RambusHypo &&
        isRead && isHit &&
        state == waitingLocMemReadResp ) {
            // done
            // do nothing
            return;
    }

    // RD Miss Dirty:
    // start --> read loc --> read far
    if (pol == enums::RambusHypo &&
        isRead && !isHit && isDirty &&
        state == waitingLocMemReadResp ) {
            orbEntry->state = farMemRead;
            orbEntry->farRdEntered = curTick();
            return;
    }

    // WR Miss Dirty:
    // start --> read loc --> loc write
    if (pol == enums::RambusHypo &&
        !isRead && !isHit && isDirty &&
        state == waitingLocMemReadResp) {
            // write it to the DRAM cache
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            return;
    }

    // RD Miss Clean & Dirty
    // start --> ... --> far read -> loc write
    if (pol == enums::RambusHypo &&
        (isRead && !isHit) &&
        state == waitingFarMemReadResp
       ) {
            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency*5);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            return;
    }

    // loc write received
    if (pol == enums::RambusHypo &&
        state == waitingLocMemWriteResp) {
            assert (!(isRead && isHit));
            // done
            // do nothing
            return;
    }

    ////////////////////////////////////////////////////////////////////////
    // BEAR Write optimized
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->state == start && !(orbEntry->owPkt->isWrite() && orbEntry->isHit)) {
            orbEntry->state = locMemRead;
            orbEntry->locRdEntered = curTick();
            DPRINTF(PolicyManager, "set: start -> locMemRead : %d\n", orbEntry->owPkt->getAddr());
            return;
    }

    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->state == start && orbEntry->owPkt->isWrite() && orbEntry->isHit) {
            orbEntry->state = locMemWrite;
            orbEntry->locRdEntered = curTick();
            DPRINTF(PolicyManager, "set: start -> locMemWrite : %d\n", orbEntry->owPkt->getAddr());
            return;
    }

    // tag ready && read && hit --> DONE
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        orbEntry->isHit) {
            DPRINTF(PolicyManager, "set: waitingLocMemReadResp -> NONE : %d\n", orbEntry->owPkt->getAddr());

            // done
            // do nothing
            return;
    }

    // tag ready && write --> loc write
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isWrite() &&
        orbEntry->state == waitingLocMemReadResp) {
            assert(!orbEntry->isHit);
            // write it to the DRAM cache
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            DPRINTF(PolicyManager, "set: waitingLocMemReadResp -> locMemWrite : %d\n", orbEntry->owPkt->getAddr());
            return;
    }

    // loc read resp ready && read && miss --> far read
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        !orbEntry->isHit) {

            orbEntry->state = farMemRead;
            orbEntry->farRdEntered = curTick();
            DPRINTF(PolicyManager, "set: waitingLocMemReadResp -> farMemRead : %d\n", orbEntry->owPkt->getAddr());
            return;
    }

    // far read resp ready && read && miss --> loc write
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingFarMemReadResp &&
        !orbEntry->isHit) {

            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency*5);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());

            // if (orbEntry->handleDirtyLine) {
            //     handleDirtyCacheLine(orbEntry);
            // }
            orbEntry->state = locMemWrite;
            orbEntry->locWrEntered = curTick();
            DPRINTF(PolicyManager, "set: waitingFarMemReadResp -> locMemWrite : %d\n", orbEntry->owPkt->getAddr());
            return;
    }

    // loc write received
    if (orbEntry->pol == enums::BearWriteOpt &&
        // orbEntry->owPkt->isRead() &&
        // !orbEntry->isHit &&
        orbEntry->state == waitingLocMemWriteResp) {
            DPRINTF(PolicyManager, "set: waitingLocMemWriteResp -> NONE : %d\n", orbEntry->owPkt->getAddr());

            // done
            // do nothing
            return;
    }
}

void
PolicyManager::handleNextState(reqBufferEntry* orbEntry)
{
    ////////////////////////////////////////////////////////////////////////
    // CascadeLakeNoPartWrs

    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->state == locMemRead) {

        // assert(!pktLocMemRead.empty());

        pktLocMemRead.push_back(orbEntry->owPkt->getAddr());

        polManStats.avgLocRdQLenEnq = pktLocMemRead.size();

        if (!locMemReadEvent.scheduled()) {
            schedule(locMemReadEvent, curTick());
        }
        return;
    }

    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        orbEntry->isHit) {
            // DONE
            // send the respond to the requestor

            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());

            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }

    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == farMemRead) {

            assert(!orbEntry->isHit);

            // do a read from far mem
            pktFarMemRead.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgFarRdQLenEnq = pktFarMemRead.size();

            if (!farMemReadEvent.scheduled()) {
                schedule(farMemReadEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        orbEntry->state == locMemWrite) {

            if (orbEntry->owPkt->isRead()) {
                assert(!orbEntry->isHit);
            }

            // do a read from far mem
            pktLocMemWrite.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgLocWrQLenEnq = pktLocMemWrite.size();


            if (!locMemWriteEvent.scheduled()) {
                schedule(locMemWriteEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::CascadeLakeNoPartWrs &&
        // orbEntry->owPkt->isRead() &&
        // !orbEntry->isHit &&
        orbEntry->state == waitingLocMemWriteResp) {
            // DONE
            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }

    ////////////////////////////////////////////////////////////////////////
    // Rambus Hypo
    if (orbEntry->pol == enums::RambusHypo &&
        orbEntry->state == locMemRead) {

        pktLocMemRead.push_back(orbEntry->owPkt->getAddr());

        polManStats.avgLocRdQLenEnq = pktLocMemRead.size();

        if (!locMemReadEvent.scheduled()) {
            schedule(locMemReadEvent, curTick());
        }
        return;
    }

    if (orbEntry->pol == enums::RambusHypo &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        orbEntry->isHit) {
            // DONE
            // send the respond to the requestor

            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());

            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }

    if (orbEntry->pol == enums::RambusHypo &&
        orbEntry->state == farMemRead) {

            assert(orbEntry->owPkt->isRead() && !orbEntry->isHit);

            // do a read from far mem
            pktFarMemRead.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgFarRdQLenEnq = pktFarMemRead.size();

            if (!farMemReadEvent.scheduled()) {
                schedule(farMemReadEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::RambusHypo &&
        orbEntry->state == locMemWrite) {

            if (orbEntry->owPkt->isRead()) {
                assert(!orbEntry->isHit);
            }

            // do a read from far mem
            pktLocMemWrite.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgLocWrQLenEnq = pktLocMemWrite.size();


            if (!locMemWriteEvent.scheduled()) {
                schedule(locMemWriteEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::RambusHypo &&
        orbEntry->state == waitingLocMemWriteResp) {
            // DONE
            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }

    ////////////////////////////////////////////////////////////////////////
    // BEAR Write Optmized
    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->state == locMemRead) {

        pktLocMemRead.push_back(orbEntry->owPkt->getAddr());

        polManStats.avgLocRdQLenEnq = pktLocMemRead.size();

        if (!locMemReadEvent.scheduled()) {
            schedule(locMemReadEvent, curTick());
        }
        return;
    }

    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == waitingLocMemReadResp &&
        orbEntry->isHit) {
            // DONE
            // send the respond to the requestor

            PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());

            accessAndRespond(orbEntry->owPkt,
                             frontendLatency + backendLatency);

            ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                                orbEntry->validEntry,
                                                orbEntry->arrivalTick,
                                                orbEntry->tagDC,
                                                orbEntry->indexDC,
                                                copyOwPkt,
                                                orbEntry->pol,
                                                orbEntry->state,
                                                orbEntry->issued,
                                                orbEntry->isHit,
                                                orbEntry->conflict,
                                                orbEntry->dirtyLineAddr,
                                                orbEntry->handleDirtyLine,
                                                orbEntry->locRdEntered,
                                                orbEntry->locRdIssued,
                                                orbEntry->locRdExit,
                                                orbEntry->locWrEntered,
                                                orbEntry->locWrIssued,
                                                orbEntry->locWrExit,
                                                orbEntry->farRdEntered,
                                                orbEntry->farRdIssued,
                                                orbEntry->farRdExit);
            delete orbEntry;

            orbEntry = ORB.at(copyOwPkt->getAddr());

            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }

    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->owPkt->isRead() &&
        orbEntry->state == farMemRead) {

            assert(!orbEntry->isHit);

            // do a read from far mem
            pktFarMemRead.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgFarRdQLenEnq = pktFarMemRead.size();

            if (!farMemReadEvent.scheduled()) {
                schedule(farMemReadEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::BearWriteOpt &&
        orbEntry->state == locMemWrite) {

            if (orbEntry->owPkt->isRead()) {
                assert(!orbEntry->isHit);
            }

            // do a read from far mem
            pktLocMemWrite.push_back(orbEntry->owPkt->getAddr());

            polManStats.avgLocWrQLenEnq = pktLocMemWrite.size();


            if (!locMemWriteEvent.scheduled()) {
                schedule(locMemWriteEvent, curTick());
            }
            return;

    }

    if (orbEntry->pol == enums::BearWriteOpt &&
        // orbEntry->owPkt->isRead() &&
        // !orbEntry->isHit &&
        orbEntry->state == waitingLocMemWriteResp) {
            // DONE
            // clear ORB
            resumeConflictingReq(orbEntry);

            return;
    }
}

void
PolicyManager::handleRequestorPkt(PacketPtr pkt)
{
    
    DPRINTF(PolicyManager, "Handling Request for Addr = %x\n", pkt->getAddr());
    DPRINTF(PolicyManager, "TSI Index = %d | BAI Index = %d | Tag = %d\n", returnBAIDC(pkt->getAddr(), pkt->getSize()), returnIndexDC(pkt->getAddr(), pkt->getSize()), returnTagDC(pkt->getAddr(), pkt->getSize()));

    //TODO: Do the compressibility check and flag setting here
    //Because the index to be used - BAI or Normal - would be decided based on that
    //TODO: DAN - Add Compressibility check here - for Writes
    //NS: Experimenting with always compressible
    if(!pkt->isRead()) {
        pkt->isCompressible = isCacheLineCompressible(pkt);
        access(pkt); //Needed for Writes
    }
    pkt->compressed_index =  (pkt->getAddr()%(blockSize*2) / blockSize);

    // MM : Pushed in the changes but will enable later
    if(!pkt->isRead()){
        if(pkt->isCompressible)
            update_LTT(pkt->getAddr(),true);
        else
            update_LTT(pkt->getAddr(),false);
    }

    //if(pkt->isRead()) access(pkt);//Experiment - Already done earlier
    DPRINTF(PolicyManager, "Request for Addr = %x => isCompressible = %d\n", pkt->getAddr(), pkt->isCompressible);

    //If prediction is correct - latencyFactor = 1
    //Else if it is incorrect and there's a hit in the other tag - latencyFactor = 2
    
    //If the prediction is incorrect
    if(pkt->isRead() && pkt->isCompressible != pkt->predCompressible) {
        if(pkt->isCompressible) { //Prediction = TSI | Actual = BAI
            DPRINTF(PolicyManager, "Checking at norm index = %d\n", returnIndexDC(pkt->getAddr(), pkt->getSize()));

            bool NormHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnIndexDC(pkt->getAddr(), pkt->getSize()), 0);
           // assert(!NormHit); //It cannot hit in TSI  - HITTING due to small address hence tag 0 for all - NEERAJ TO CHECK

            bool BaiHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnBAIDC(pkt->getAddr(), pkt->getSize()), 1);
            if(BaiHit) {
                pkt->latencyFactor = 2;
                DPRINTF(PolicyManager, "For Misprediction of Request Addr = %d | Got BAI HIT - Setting latency factor of 2\n", pkt->getAddr());
                //TODO NEERAJ - ADD STAT HERE WRONG_PRED_HIT++
                polManStats.numWrongPredHIT++;
            }else {
                //TODO NEERAJ - ADD STAT HERE WRONG_PRED_MISS++
                polManStats.numWrongPredMISS++;
            }

            if(NormHit || BaiHit)
                update_MAPI(pkt->getAddr(),true);
            else
                update_MAPI(pkt->getAddr(),false);
        }
        else { //Prediction = BAI | Actual = TSI
            bool BaiHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnBAIDC(pkt->getAddr(), pkt->getSize()), 1);
            //assert(!BaiHit); //It cannot hit in TSI 

            
            bool NormHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnIndexDC(pkt->getAddr(), pkt->getSize()), 0);
            if(NormHit) {
                pkt->latencyFactor = 2;
                DPRINTF(PolicyManager, "For Misprediction of Request Addr = %d | Got Normal HIT - Setting latency factor of 2\n", pkt->getAddr());
                //TODO NEERAJ - ADD STAT HERE WRONG_PRED_HIT++
                polManStats.numWrongPredHIT++;
            }else {
                //TODO NEERAJ - ADD STAT HERE WRONG_PRED_MISS++
                polManStats.numWrongPredMISS++;

            }

            if(NormHit || BaiHit)
                update_MAPI(pkt->getAddr(),true);
            else
                update_MAPI(pkt->getAddr(),false);
        }

    }
    
    //Also update_MAPI when DICE prediction is correct. Basically update it for every request - CHECK MANU?
    if(pkt->isRead()) {
        bool BaiHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnBAIDC(pkt->getAddr(), pkt->getSize()), 1);
        bool NormHit = checkHit(pkt, returnTagDC(pkt->getAddr(), pkt->getSize()), returnIndexDC(pkt->getAddr(), pkt->getSize()), 0);
        if(NormHit || BaiHit) update_MAPI(pkt->getAddr(),true);
        else update_MAPI(pkt->getAddr(),false);
    }

    reqBufferEntry* orbEntry = new reqBufferEntry(
                                true, curTick(),
                                returnTagDC(pkt->getAddr(), pkt->getSize()),
                                (/*pkt->predCompressible || */pkt->isCompressible)? returnBAIDC(pkt->getAddr(), pkt->getSize()) : returnIndexDC(pkt->getAddr(), pkt->getSize()), //TODO: NEERAJ - Also use isCompressible for writes
                                pkt,
                                locMemPolicy, start,
                                false, false, false,
                                -1, false,
                                MaxTick, MaxTick, MaxTick,
                                MaxTick, MaxTick, MaxTick,
                                MaxTick, MaxTick, MaxTick
                            );

    ORB.emplace(pkt->getAddr(), orbEntry);

    polManStats.avgORBLen = ORB.size();
    polManStats.avgLocRdQLenStrt = countLocRdInORB();
    polManStats.avgFarRdQLenStrt = countFarRdInORB();
    polManStats.avgLocWrQLenStrt = countLocWrInORB();
    polManStats.avgFarWrQLenStrt = countFarWr();

    Addr addr = pkt->getAddr();
    unsigned burst_size = locBurstSize;
    unsigned size = std::min((addr | (burst_size - 1)) + 1,
                              addr + pkt->getSize()) - addr;

    if(pkt->isRead()) {
        polManStats.bytesReadSys += size;
        polManStats.readPktSize[ceilLog2(size)]++;
        polManStats.readReqs++;
    } else {
        polManStats.bytesWrittenSys += size;
        polManStats.writePktSize[ceilLog2(size)]++;
        polManStats.writeReqs++;
    }

    if (pkt->isWrite()) {

        PacketPtr copyOwPkt = new Packet(orbEntry->owPkt,
                                             false,
                                             orbEntry->owPkt->isRead());
        //Update_752: If response/ACK is not required, only access would suffice
        if(pkt->needsResponse()) {

        accessAndRespond(orbEntry->owPkt,
                         frontendLatency + backendLatency);
        }
        else access(pkt);

        ORB.at(copyOwPkt->getAddr()) = new reqBufferEntry(
                                            orbEntry->validEntry,
                                            orbEntry->arrivalTick,
                                            orbEntry->tagDC,
                                            orbEntry->indexDC,
                                            copyOwPkt,
                                            orbEntry->pol,
                                            orbEntry->state,
                                            orbEntry->issued,
                                            orbEntry->isHit,
                                            orbEntry->conflict,
                                            orbEntry->dirtyLineAddr,
                                            orbEntry->handleDirtyLine,
                                            orbEntry->locRdEntered,
                                            orbEntry->locRdIssued,
                                            orbEntry->locRdExit,
                                            orbEntry->locWrEntered,
                                            orbEntry->locWrIssued,
                                            orbEntry->locWrExit,
                                            orbEntry->farRdEntered,
                                            orbEntry->farRdIssued,
                                            orbEntry->farRdExit);
        delete orbEntry;

        orbEntry = ORB.at(copyOwPkt->getAddr());
    }

    checkHitOrMiss(orbEntry, pkt->isCompressible/*pkt->predCompressible*/);
    //NS: If Read and predicted compressed and it is not a HIT, then check the normal tags, and if a HIT is found, then set 2x latency.
    //Done already above
    /*if(pkt->isRead() && !orbEntry->isHit && pkt->predCompressible) {
        Addr norm_idx = returnIndexDC(pkt->getAddr(), pkt->getSize());
        bool currValid = tagMetadataStore.at(norm_idx).validLine;
        bool currDirty = tagMetadataStore.at(norm_idx).dirtyLine;


        bool hit = currValid && (orbEntry->tagDC == tagMetadataStore.at(norm_idx).tagDC);
        if(hit) {
            orbEntry->owPkt->latencyFactor = 2;
            // MM : Made the updates, will enable later doing some testing
            //update_LTT(pkt->getAddr(),false);
        }

        DPRINTF(PolicyManager, "NS: For Address %d | Checking normal tag | isHit = %d\n", orbEntry->owPkt->getAddr(), hit);

    }
    else */ 
    if (pkt->isRead() && orbEntry->isHit && pkt->isCompressible && pkt->compressed_index == 1) { //TODO NEERAJ - Try updating this logic
        DPRINTF(PolicyManager, "NS: For Address %d | HIT for compressed index 1 | Setting latencyFactor as 0\n", orbEntry->owPkt->getAddr());
        //TODO NEERAJ - ADD STAT HERE BAI_HIT++
        polManStats.numIndexBAIHIT++;
        orbEntry->owPkt->latencyFactor = 0;
    }
    //NS: Just to check if this change is reaching accessAndRespond - Verified
    //orbEntry->owPkt->latencyFactor = 0;

    DPRINTF(PolicyManager, "NS: For Address %d | Predicted Compressible = %d| IsHit = %d\n", orbEntry->owPkt->getAddr(), pkt->predCompressible, orbEntry->isHit);

    if (checkDirty(orbEntry->owPkt->getAddr(), (/*orbEntry->owPkt->predCompressible || */orbEntry->owPkt->isCompressible)) && !orbEntry->isHit) {
        orbEntry->dirtyLineAddr = tagMetadataStore.at(orbEntry->indexDC).farMemAddr;
        orbEntry->handleDirtyLine = true;
    }

    //NS: Check if data is compressible
    //NS: If it is then update tag and metadata for BAI else update the usual one
    //NS: OR Can maintain a single tag too

    //NS: To determine compressibility, read the data here
    //NS: For writes the access is done so data should be available
    //NS: What about Reads? - Handled using predictor
    //NS: Call compression logic for Writes
    //TODO DANIEL - Do it at before the orbEntry is created - search by your name

    //NS: Experimenting with always compressible
    //if(orbEntry->owPkt->getAddr()%128 == 0) orbEntry->owPkt->isCompressible = true;

    //Calculate the Compressible data index.
    //For e.g. - A0 is at compressed index 0 while A1 is at compressed index 1 in the set index 0
    int compressed_index =  (orbEntry->owPkt->getAddr()%(blockSize*2) / blockSize);

    //If data is compressible then update the BAI tag else the usual TSI tag

    //TODO NEERAJ - What if Read was predicted to be compressible and hence tagBAI was updated but in reality it was not compressible and hence normal tags needed to be updated?
    //How about maintaining a single - tagBAI - like structure. Have a flag uncompressed - If the line is uncompressed, the flag will be unset. Can use any index for storage of such lines
    //Functions like checkDirty above have been done for predicted index which could mess up the simulation. Handle such cases too
    //Now when allocating a new compressed line, a check to see if the other half is compressed or not would be required
    //Also, when allocating say A0 in set 0, compressed index 0, check the compressed index 1 - if it holds A1 then okay, else evict that index 1.

    //Solved the above issue by :
    //Accessing the data and knowing if it's going to be compressible
    //And then using this knowledge to update the correct tag
    //PredCompressible is used only to deteremine the response latency from the DRAM cache

    //TODO NEERAJ - Also use predCompressible to check which tag to update - This could mess things - So update only the correct tag

    if(orbEntry->owPkt->isCompressible /*|| orbEntry->owPkt->predCompressible*/) {
        //Updating BAI Tag and Metadata
        tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).tagDC = orbEntry->tagDC;
        tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).indexDC = orbEntry->indexDC;
        tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).validLine = true;
        DPRINTF(PolicyManager, "For Address %d | Adding bai tag for set index = %d and compressed index = %d\n", orbEntry->owPkt->getAddr(), orbEntry->indexDC, compressed_index);

        //Evict from the other tag
        tagMetadataStore.at(orbEntry->indexDC).validLine = false;
    } else {
        // Updating Normal Tag & Metadata
        tagMetadataStore.at(orbEntry->indexDC).tagDC = orbEntry->tagDC;
        tagMetadataStore.at(orbEntry->indexDC).indexDC = orbEntry->indexDC;
        tagMetadataStore.at(orbEntry->indexDC).validLine = true;
        DPRINTF(PolicyManager, "For Address %d | Adding normal tag for set index = %d\n", orbEntry->owPkt->getAddr(), orbEntry->indexDC);

        //Evict from the other tag
        tagBaiMetadataStore.at(orbEntry->indexDC).at(0).validLine = false;
        tagBaiMetadataStore.at(orbEntry->indexDC).at(1).validLine = false;
    }
    //NS: TODO NEERAJ - Handle tag evictions too - DONE

    if (orbEntry->owPkt->isRead()) {
        if(orbEntry->owPkt->isCompressible) {
            if (orbEntry->isHit) {
                tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).dirtyLine =
                tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).dirtyLine;
            } else {
                tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).dirtyLine = false;
            }
        }else {
            if (orbEntry->isHit) {
                tagMetadataStore.at(orbEntry->indexDC).dirtyLine =
                tagMetadataStore.at(orbEntry->indexDC).dirtyLine;
            } else {
                tagMetadataStore.at(orbEntry->indexDC).dirtyLine = false;
            }
        }
    } else {
        if(orbEntry->owPkt->isCompressible) tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).dirtyLine = true;
        else tagMetadataStore.at(orbEntry->indexDC).dirtyLine = true;

    }


    if(orbEntry->owPkt->isCompressible) tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).farMemAddr = orbEntry->owPkt->getAddr();
    else tagMetadataStore.at(orbEntry->indexDC).farMemAddr = orbEntry->owPkt->getAddr();
}

bool
PolicyManager::checkConflictInDramCache(PacketPtr pkt)
{
    unsigned indexDC = (/*pkt->predCompressible || */pkt->isCompressible)? returnBAIDC(pkt->getAddr(), pkt->getSize()) : returnIndexDC(pkt->getAddr(), pkt->getSize());
    //NS: If prediction is incompressible - conflicts work as usual 0 but shouldn't these conflicts also check for address anyway? 
    //NS: If prediction is compressible - If the compressed index is different then is that considered a conflict? Assuming A1 is present whenevr A0 is present, then yes. Else no.
    //Always assuming here that if a compressed line is valid, both the compressed indexes are present

    for (auto e = ORB.begin(); e != ORB.end(); ++e) {
        if (indexDC == e->second->indexDC && e->second->validEntry) {

            e->second->conflict = true;

            return true;
        }
    }
    return false;
}

//NS: Only checking based on addr and index
bool
PolicyManager::checkHit(PacketPtr pkt, Addr tagDC, Addr indexDC, bool compressed)
{
    // access the tagMetadataStore data structure to
    // check if it's hit or miss
    int compressed_index =  (pkt->getAddr()%(2*blockSize) / blockSize);
    
    bool currValid;
    bool currDirty;

    if (compressed) {
        currValid = tagBaiMetadataStore.at(indexDC).at(compressed_index).validLine;
        return (currValid && (tagDC == tagBaiMetadataStore.at(indexDC).at(compressed_index).tagDC));
    } else {
        currValid = tagMetadataStore.at(indexDC).validLine;
        return (currValid && (tagDC == tagMetadataStore.at(indexDC).tagDC));
    }
    return false;
}

void
PolicyManager::checkHitOrMiss(reqBufferEntry* orbEntry, bool compressed)
{
    // access the tagMetadataStore data structure to
    // check if it's hit or miss
    int compressed_index =  (orbEntry->owPkt->getAddr()%(2*blockSize) / blockSize);

    bool currValid;
    bool currDirty;

    //NS:
    if (compressed) {
        currValid = tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).validLine;
        currDirty = tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).dirtyLine;
        orbEntry->isHit = currValid && (orbEntry->tagDC == tagBaiMetadataStore.at(orbEntry->indexDC).at(compressed_index).tagDC);
    } else {
        currValid = tagMetadataStore.at(orbEntry->indexDC).validLine;
        currDirty = tagMetadataStore.at(orbEntry->indexDC).dirtyLine;
        orbEntry->isHit = currValid && (orbEntry->tagDC == tagMetadataStore.at(orbEntry->indexDC).tagDC);
    }

    // orbEntry->isHit = alwaysHit;

    if (orbEntry->isHit) {

        polManStats.numTotHits++;

        if (orbEntry->owPkt->isRead()) {
            polManStats.numRdHit++;
            if (currDirty) {
                polManStats.numRdHitDirty++;
            } else {
                polManStats.numRdHitClean++;
            }
        } else {
            polManStats.numWrHit++;
            if (currDirty) {
                polManStats.numWrHitDirty++;
            } else {
                polManStats.numWrHitClean++;
            }
        }

    } else {

        //update_MAPI(orbEntry->owPkt->getAddr(),false);

        polManStats.numTotMisses++;

        if (currValid) {
            polManStats.numHotMisses++;
        } else {
            polManStats.numColdMisses++;
            numColdMisses++;
        }

        if (orbEntry->owPkt->isRead()) {
            if (currDirty && currValid) {
                polManStats.numRdMissDirty++;
            } else {
                polManStats.numRdMissClean++;
            }
        } else {
            if (currDirty && currValid) {
                polManStats.numWrMissDirty++;
            } else {
                polManStats.numWrMissClean++;
            }

        }
    }

    // if ( numColdMisses >= (unsigned)(cacheWarmupRatio * dramCacheSize/blockSize) && !resetStatsWarmup ) {
    //    std::cout << curTick() << " --------------------------1\n";
    //    exitSimLoopNow("cacheIsWarmedup");
    //    std::cout << curTick() << " --------------------------2\n";
    //    resetStatsWarmup = true;
    // }
}

bool
PolicyManager::checkDirty(Addr addr, bool compressed)
{
    Addr index = compressed? returnBAIDC(addr, blockSize) : returnIndexDC(addr, blockSize);
    int compressed_index = addr%(2*blockSize)/blockSize;

    if(compressed) {
            return (tagBaiMetadataStore.at(index).at(compressed_index).validLine &&
           tagBaiMetadataStore.at(index).at(compressed_index).dirtyLine);
    }
    else {
    return (tagMetadataStore.at(index).validLine &&
           tagMetadataStore.at(index).dirtyLine);
    }
    // return alwaysDirty;
}

void
PolicyManager::accessAndRespond(PacketPtr pkt, Tick static_latency)
{
    DPRINTF(PolicyManager, "Responding to Address %d \n", pkt->getAddr());
    DPRINTF(PolicyManager, "latencyFactor = %d \n", pkt->latencyFactor);

    bool needsResponse = pkt->needsResponse();
    // do the actual memory access which also turns the packet into a
    // response
    panic_if(!getAddrRange().contains(pkt->getAddr()),
             "Can't handle address range for packet %s\n", pkt->print());
    access(pkt);

    // turn packet around to go back to requestor if response expected
    //assert(needsResponse); //Experiment
    //if (needsResponse) {
        // access already turned the packet into a response
        assert(pkt->isResponse());
        // response_time consumes the static latency and is charged also
        // with headerDelay that takes into account the delay provided by
        // the xbar and also the payloadDelay that takes into account the
        // number of data beats.
        Tick response_time = curTick() + static_latency/**pkt->latencyFactor*/ + pkt->headerDelay +
                             pkt->payloadDelay; //NS
        // Here we reset the timing of the packet before sending it out.
        pkt->headerDelay = pkt->payloadDelay = 0;

        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pkt, response_time);
    //}
    // else {
    //     // @todo the packet is going to be deleted, and the MemPacket
    //     // is still having a pointer to it
    //     pendingDelete.reset(pkt);
    // }

    DPRINTF(PolicyManager, "Done\n");

    return;
}

PacketPtr
PolicyManager::getPacket(Addr addr, unsigned size, const MemCmd& cmd,
                   Request::FlagsType flags)
{
    // Create new request
    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                                               0);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)0) << 2);

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, cmd);

    uint8_t* pkt_data = new uint8_t[req->getSize()];

    pkt->dataDynamic(pkt_data);

    if (cmd.isWrite()) {
        std::fill_n(pkt_data, req->getSize(), (uint8_t)0);
    }

    return pkt;
}

void
PolicyManager::sendRespondToRequestor(PacketPtr pkt, Tick static_latency)
{
    PacketPtr copyOwPkt = new Packet(pkt,
                                     false,
                                     pkt->isRead());
    copyOwPkt->makeResponse();

    Tick response_time = curTick() + static_latency + copyOwPkt->headerDelay + copyOwPkt->payloadDelay;
    // Here we reset the timing of the packet before sending it out.
    copyOwPkt->headerDelay = copyOwPkt->payloadDelay = 0;

    // queue the packet in the response queue to be sent out after
    // the static latency has passed
    port.schedTimingResp(copyOwPkt, response_time);

}

bool
PolicyManager::resumeConflictingReq(reqBufferEntry* orbEntry)
{
    bool conflictFound = false;

    if (orbEntry->owPkt->isWrite()) {
        isInWriteQueue.erase(orbEntry->owPkt->getAddr());
    }

    logStatsPolMan(orbEntry);

    for (auto e = CRB.begin(); e != CRB.end(); ++e) {

        auto entry = *e;

        if (((/*entry.second->predCompressible || */entry.second->isCompressible)? returnBAIDC(entry.second->getAddr(), entry.second->getSize()) : returnIndexDC(entry.second->getAddr(), entry.second->getSize()))
            == orbEntry->indexDC) {

                conflictFound = true;

                Addr confAddr = entry.second->getAddr();

                ORB.erase(orbEntry->owPkt->getAddr());

                delete orbEntry->owPkt;

                delete orbEntry;

                handleRequestorPkt(entry.second);

                ORB.at(confAddr)->arrivalTick = entry.first;

                DPRINTF(PolicyManager, "Resuming conflict at Addr %d\n", confAddr);

                CRB.erase(e);

                checkConflictInCRB(ORB.at(confAddr));

                // pktLocMemRead.push_back(confAddr);

                // polManStats.avgLocRdQLenEnq = pktLocMemRead.size();

                setNextState(ORB.at(confAddr));

                handleNextState(ORB.at(confAddr));

                break;
        }

    }

    if (!conflictFound) {

        ORB.erase(orbEntry->owPkt->getAddr());

        delete orbEntry->owPkt;

        delete orbEntry;

        if (retryLLC) {
            DPRINTF(PolicyManager, "retryLLC: sent\n");
            retryLLC = false;
            port.sendRetryReq();
        } else {
            if (drainState() == DrainState::Draining && ORB.empty() &&
                pktFarMemWrite.empty()) {
                DPRINTF(Drain, "PolicyManager done draining\n");
                signalDrainDone();
            }
        }
    }

    return conflictFound;
}

void
PolicyManager::checkConflictInCRB(reqBufferEntry* orbEntry)
{
    for (auto e = CRB.begin(); e != CRB.end(); ++e) {

        auto entry = *e;

        if (((/*orbEntry->owPkt->predCompressible ||*/ orbEntry->owPkt->isCompressible)? returnBAIDC(entry.second->getAddr(),entry.second->getSize()) : returnIndexDC(entry.second->getAddr(),entry.second->getSize()))
            == orbEntry->indexDC) {
                orbEntry->conflict = true;
                break;
        }
    }
}

unsigned
PolicyManager::countLocRdInORB()
{
    unsigned count =0;
    for (auto i : ORB) {
        if (i.second->state == locMemRead) {
            count++;
        }
    }
    return count;
}

unsigned
PolicyManager::countFarRdInORB()
{
    unsigned count =0;
    for (auto i : ORB) {
        if (i.second->state == farMemRead) {
            count++;
        }
    }
    return count;
}

unsigned
PolicyManager::countLocWrInORB()
{
    unsigned count =0;
    for (auto i : ORB) {
        if (i.second->state == locMemWrite) {
            count++;
        }
    }
    return count;
}

unsigned
PolicyManager::countFarWr()
{
    return pktFarMemWrite.size();
}

AddrRangeList
PolicyManager::getAddrRanges()
{
    return farReqPort.getAddrRanges();
}

bool PolicyManager::read_LTT(Addr request_addr) {

    // Apply hash function and take modulo to fit into the ltt_table size
    // TODO : MM : Assumed a page size of 4096 for now, will replace it with page size

    Addr page_number = request_addr>>12;
    bool ltt_value = last_time_table[ltt_hash_fn(page_number) % ltt_table_size];
    DPRINTF(PolicyManager, "read_LTT: Addr 0x%x, Page no. 0x%0x, ltt_value %0d \n",request_addr,page_number,ltt_value);

    return ltt_value;
}

void PolicyManager::update_LTT(Addr request_addr, bool is_read_data_compressible=false) {

    // Apply hash function and take modulo to fit into the ltt_table size
    // TODO : MM : Assumed a page size of 4096 for now, will replace it with page size

    Addr page_number = request_addr>>12;
    Addr table_idx = ltt_hash_fn(page_number) % ltt_table_size;
    bool predicted_compressible = last_time_table[table_idx];

    if(predicted_compressible == is_read_data_compressible)
        polManStats.numOfTimesDicePredictorCorrect++;
    else
        polManStats.numOfTimesDicePredictorIncorrect++;

    last_time_table[table_idx] = is_read_data_compressible;
    DPRINTF(PolicyManager, "update_LTT: Addr 0x%x, Page no. 0x%0x, table idx %0d, predicted_compressible %0d, actual_compressible %0d \n",request_addr,page_number,table_idx,predicted_compressible,is_read_data_compressible);

    return;
}

bool PolicyManager::read_MAPI(Addr request_addr) {

    // Apply hash function and take modulo to fit into the inst_based_MAP_table_size

    bool bypass_dcache;
    Addr mapi_table_index;
    mapi_table_index = inst_based_MAP_hash_fn(request_addr) % inst_based_MAP_table_size;
    int mapi_cnt_value = inst_based_MAP_table[mapi_table_index];
    bypass_dcache = mapi_cnt_value/int(std::pow(2, inst_based_MAP_bit_vector_size-1));
    DPRINTF(PolicyManager, "read_MAPI: Addr 0x%x, mapi_cnt_value %0d, mapi_table_index %0d, bypass_dcache %0d \n",request_addr,mapi_cnt_value,mapi_table_index,bypass_dcache);

    return bypass_dcache;
}

void PolicyManager::update_MAPI_count(Addr request_addr, bool is_dram_cache_hit=false) {

    Addr mapi_table_index;

    mapi_table_index = inst_based_MAP_hash_fn(request_addr) % inst_based_MAP_table_size;

    if(inst_based_MAP_table.find(mapi_table_index) == inst_based_MAP_table.end())
        inst_based_MAP_table[mapi_table_index] = 1;

    if(!is_dram_cache_hit){
        if(inst_based_MAP_table[mapi_table_index] < (int(std::pow(2, inst_based_MAP_bit_vector_size))-1))
            inst_based_MAP_table[mapi_table_index] += 1;
    }else{
        if(inst_based_MAP_table[mapi_table_index] > 0)
            inst_based_MAP_table[mapi_table_index] -= 1;
    } 

}

void PolicyManager::update_MAPI(Addr request_addr, bool is_dram_cache_hit=false) {

    // Apply hash function and take modulo to fit into the inst_based_MAP_table_size

    Addr mapi_table_index;
    mapi_table_index = inst_based_MAP_hash_fn(request_addr) % inst_based_MAP_table_size;

    bool predicted_bypass_dcache = inst_based_MAP_table[mapi_table_index]/int(std::pow(2, inst_based_MAP_bit_vector_size-1));

    if(predicted_bypass_dcache != is_dram_cache_hit)
        polManStats.numOfTimesBypassDcachePredictorCorrect++;
    else
        polManStats.numOfTimesBypassDcachePredictorIncorrect++;

    update_MAPI_count(request_addr,is_dram_cache_hit);
    DPRINTF(PolicyManager, "update_MAPI: Addr 0x%x, predicted_bypass_dcache %0d, mapi_table_index %0d, is_dram_cache_hit %0d \n",request_addr,predicted_bypass_dcache,mapi_table_index,is_dram_cache_hit);

    return;
}

Addr
PolicyManager::returnIndexDC(Addr request_addr, unsigned size)
{
    int index_bits = ceilLog2(dramCacheSize/blockSize);
    int block_bits = ceilLog2(size);
    return bits(request_addr, block_bits + index_bits-1, block_bits);
}

//NS: Adding for BAI
Addr
PolicyManager::returnBAIDC(Addr request_addr, unsigned size)
{
    int index_bits = ceilLog2(dramCacheSize/blockSize);
    int block_bits = ceilLog2(size);
    int numBlocks = ceil(dramCacheSize/blockSize);

    if((request_addr%128 == 0) && (request_addr%(2*numBlocks) == 0) || (request_addr%128 == 1) && (request_addr%(2*numBlocks) == 1)) return bits(request_addr, block_bits + index_bits-1, block_bits); //A0 or A9
    else if((request_addr%128 == 1) && (request_addr%(2*numBlocks) == 0)) return (bits(request_addr, block_bits + index_bits-1, block_bits) - 1); //A1
    else if((request_addr%128 == 0) && (request_addr%(2*numBlocks) == 1)) return (bits(request_addr, block_bits + index_bits-1, block_bits) + 1); //A8

    //return bits(request_addr, block_bits + index_bits-1, block_bits)/2; //NS TODO NEERAJ: For now using NSI by dividing the index by 2 
}

Addr
PolicyManager::returnTagDC(Addr request_addr, unsigned size)
{
    int index_bits = ceilLog2(dramCacheSize/blockSize);
    int block_bits = ceilLog2(size);
    return bits(request_addr, addrSize-1, (index_bits+block_bits));
}

void
PolicyManager::handleDirtyCacheLine(reqBufferEntry* orbEntry)
{
    assert(orbEntry->dirtyLineAddr != -1);

    // create a new request packet
    PacketPtr wbPkt = getPacket(orbEntry->dirtyLineAddr,
                                orbEntry->owPkt->getSize(),
                                MemCmd::WriteReq);
    uint8_t* data_recv = wbPkt->getPtr<uint8_t>();

    pktFarMemWrite.push_back(std::make_pair(curTick(), wbPkt));

    polManStats.avgFarWrQLenEnq = pktFarMemWrite.size();

    if (!farMemWriteEvent.scheduled()) {
            schedule(farMemWriteEvent, curTick());
    }

    polManStats.numWrBacks++;
}

bool
PolicyManager::isCacheLineCompressible(PacketPtr pkt)
{
    uint64_t* data_ptr = pkt->getPtr<uint64_t>();
    size_t data_size = pkt->getSize();
    // assert(data_size % 8 == 0);

    Cycles compression_lat = Cycles(0);
    Cycles decompression_lat = Cycles(0);
    std::unique_ptr<compression::Base::CompressionData> compressed_data;
    size_t bdi_compressed_bits = 0;
    size_t fpc_compressed_bits = 0;

    // printf("COMPRESSING WTIH BDI\n");
    compressed_data =
        bdi_compressor->compress(data_ptr, compression_lat, decompression_lat);
    bdi_compressed_bits = compressed_data->getSizeBits();
    // printf("Compressed bits: %lu\n", bdi_compressed_bits);


    // printf("COMPRESSING WITH FPC\n");
    compressed_data =
        fpc_compressor->compress(data_ptr, compression_lat, decompression_lat);
    fpc_compressed_bits = compressed_data->getSizeBits();
    // printf("Compressed bits: %lu\n", fpc_compressed_bits);

    const size_t dice_compression_threshold_bits = 36 * 8; // 36 bytes * 8 bits

    // compressible_checks_counter++;
    if (bdi_compressed_bits < dice_compression_threshold_bits ||
        fpc_compressed_bits < dice_compression_threshold_bits) {
            // printf("CACHE LINE IS COMPRESSIBLE\n");
            // num_compressible_counter++;
            // if (compressible_checks_counter % 1000 == 0) {
            //     printf("Compressed / compressible checks = %lu / %lu\n",
            //         num_compressible_counter, compressible_checks_counter);
            // }
            return true;
    }
    // if (compressible_checks_counter % 1000 == 0) {
    //     printf("Compressed / compressible checks = %lu / %lu\n",
    //         num_compressible_counter, compressible_checks_counter);
    // }
    return false;
}

void
PolicyManager::logStatsPolMan(reqBufferEntry* orbEntry)
{
    polManStats.totPktsServiceTime += ((curTick() - orbEntry->arrivalTick)/1000);
    polManStats.totPktsORBTime += ((curTick() - orbEntry->locRdEntered)/1000);
    polManStats.totTimeFarRdtoSend += ((orbEntry->farRdIssued - orbEntry->farRdEntered)/1000);
    polManStats.totTimeFarRdtoRecv += ((orbEntry->farRdExit - orbEntry->farRdIssued)/1000);
    polManStats.totTimeInLocRead += ((orbEntry->locRdExit - orbEntry->locRdEntered)/1000);
    polManStats.totTimeInLocWrite += ((orbEntry->locWrExit - orbEntry->locWrEntered)/1000);
    polManStats.totTimeInFarRead += ((orbEntry->farRdExit - orbEntry->farRdEntered)/1000);

}


void
PolicyManager::ReqPortPolManager::recvReqRetry()
{
    if (this->name() == "system.mem_ctrl.loc_req_port") {
        polMan.locMemRecvReqRetry();
    }
    if (this->name() == "system.mem_ctrl.far_req_port") {
        polMan.farMemRecvReqRetry();
    }
}

bool
PolicyManager::ReqPortPolManager::recvTimingResp(PacketPtr pkt)
{
    if (this->name() == "system.mem_ctrl.loc_req_port") {
        return polMan.locMemRecvTimingResp(pkt);
    } else if (this->name() == "system.mem_ctrl.far_req_port") {
        return polMan.farMemRecvTimingResp(pkt);
    } else {
        std::cout << "Port name error, fix it!\n";
        return false;
    }
}

PolicyManager::PolicyManagerStats::PolicyManagerStats(PolicyManager &_polMan)
    : statistics::Group(&_polMan),
    polMan(_polMan),

/////
    ADD_STAT(readReqs, statistics::units::Count::get(),
             "Number of read requests accepted"),
    ADD_STAT(writeReqs, statistics::units::Count::get(),
             "Number of write requests accepted"),

    ADD_STAT(servicedByWrQ, statistics::units::Count::get(),
             "Number of controller read bursts serviced by the write queue"),
    ADD_STAT(mergedWrBursts, statistics::units::Count::get(),
             "Number of controller write bursts merged with an existing one"),

    ADD_STAT(numRdRetry, statistics::units::Count::get(),
             "Number of times read queue was full causing retry"),
    ADD_STAT(numWrRetry, statistics::units::Count::get(),
             "Number of times write queue was full causing retry"),

    ADD_STAT(readPktSize, statistics::units::Count::get(),
             "Read request sizes (log2)"),
    ADD_STAT(writePktSize, statistics::units::Count::get(),
             "Write request sizes (log2)"),

    ADD_STAT(bytesReadWrQ, statistics::units::Byte::get(),
             "Total number of bytes read from write queue"),
    ADD_STAT(bytesReadSys, statistics::units::Byte::get(),
             "Total read bytes from the system interface side"),
    ADD_STAT(bytesWrittenSys, statistics::units::Byte::get(),
             "Total written bytes from the system interface side"),

    ADD_STAT(avgRdBWSys, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average system read bandwidth in Byte/s"),
    ADD_STAT(avgWrBWSys, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average system write bandwidth in Byte/s"),

    ADD_STAT(totGap, statistics::units::Tick::get(),
             "Total gap between requests"),

    ADD_STAT(avgGap, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average gap between requests"),

    ADD_STAT(numOfTimesDicePredictorCorrect, statistics::units::Tick::get(),
             "Number of correct predictions by the DICE predictor"),

    ADD_STAT(numOfTimesDicePredictorIncorrect, statistics::units::Tick::get(),
             "Number of incorrect predictions by the DICE predictor"),

    ADD_STAT(accuracyOfDicePredictor, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Accuracy of the DICE predictor"),

    ADD_STAT(numOfTimesBypassDcachePredictorCorrect, statistics::units::Tick::get(),
             "Number of correct predictions by the bypass dcache predictor"),

    ADD_STAT(numOfTimesBypassDcachePredictorIncorrect, statistics::units::Tick::get(),
             "Number of incorrect predictions by the bypass dcache predictor"),          
    
    ADD_STAT(accuracyOfBypassDcachePredictor, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Accuracy of the Bypass Dcache predictor"),

    ADD_STAT(avgORBLen, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average ORB length"),
    ADD_STAT(avgLocRdQLenStrt, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average local read queue length"),
    ADD_STAT(avgLocWrQLenStrt, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average local write queue length"),
    ADD_STAT(avgFarRdQLenStrt, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average far read queue length"),
    ADD_STAT(avgFarWrQLenStrt, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average far write queue length"),

    ADD_STAT(avgLocRdQLenEnq, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average local read queue length when enqueuing"),
    ADD_STAT(avgLocWrQLenEnq, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average local write queue length when enqueuing"),
    ADD_STAT(avgFarRdQLenEnq, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average far read queue length when enqueuing"),
    ADD_STAT(avgFarWrQLenEnq, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average far write queue length when enqueuing"),

    ADD_STAT(numWrBacks,
            "Total number of write backs from DRAM cache to main memory"),
    ADD_STAT(totNumConf,
            "Total number of packets conflicted on DRAM cache"),
    ADD_STAT(totNumORBFull,
            "Total number of packets ORB full"),
    ADD_STAT(totNumCRBFull,
            "Total number of packets conflicted yet couldn't "
            "enter confBuffer"),

    ADD_STAT(maxNumConf,
            "Maximum number of packets conflicted on DRAM cache"),

    ADD_STAT(sentLocRdPort,
             "stat"),
    ADD_STAT(sentLocWrPort,
             "stat"),
    ADD_STAT(failedLocRdPort,
             "stat"),
    ADD_STAT(failedLocWrPort,
             "stat"),
    ADD_STAT(recvdRdPort,
             "stat"),
    ADD_STAT(sentFarRdPort,
             "stat"),
    ADD_STAT(sentFarWrPort,
             "stat"),
    ADD_STAT(failedFarRdPort,
             "stat"),
    ADD_STAT(failedFarWrPort,
             "stat"),

    ADD_STAT(totPktsServiceTime,
            "stat"),
    ADD_STAT(totPktsORBTime,
            "stat"),
    ADD_STAT(totTimeFarRdtoSend,
            "stat"),
    ADD_STAT(totTimeFarRdtoRecv,
            "stat"),
    ADD_STAT(totTimeFarWrtoSend,
            "stat"),
    ADD_STAT(totTimeInLocRead,
            "stat"),
    ADD_STAT(totTimeInLocWrite,
            "stat"),
    ADD_STAT(totTimeInFarRead,
            "stat"),

    ADD_STAT(numTotHits,
            "stat"),
    ADD_STAT(numTotMisses,
            "stat"),
    ADD_STAT(numColdMisses,
            "stat"),
    ADD_STAT(numHotMisses,
            "stat"),
    ADD_STAT(numRdMissClean,
            "stat"),
    ADD_STAT(numRdMissDirty,
            "stat"),
    ADD_STAT(numRdHit,
            "stat"),
    ADD_STAT(numWrMissClean,
            "stat"),
    ADD_STAT(numWrMissDirty,
            "stat"),
    ADD_STAT(numWrHit,
            "stat"),
    ADD_STAT(numRdHitDirty,
            "stat"),
    ADD_STAT(numRdHitClean,
            "stat"),
    ADD_STAT(numWrHitDirty,
            "stat"),
    ADD_STAT(numWrHitClean,
            "stat"),
    ADD_STAT(numWrongPredHIT,
            "Incorrect DICE Prediction but HIT in the other index"),        
    ADD_STAT(numWrongPredMISS,
            "Incorrect DICE Prediction and MISS in both the indices"),
    ADD_STAT(numIndexBAIHIT,
            "Compressed data at index 1 got a HIT - BW Improvement")
{
}

void
PolicyManager::PolicyManagerStats::regStats()
{
    using namespace statistics;

    avgORBLen.precision(4);
    avgLocRdQLenStrt.precision(2);
    avgLocWrQLenStrt.precision(2);
    avgFarRdQLenStrt.precision(2);
    avgFarWrQLenStrt.precision(2);

    avgLocRdQLenEnq.precision(2);
    avgLocWrQLenEnq.precision(2);
    avgFarRdQLenEnq.precision(2);
    avgFarWrQLenEnq.precision(2);

    readPktSize.init(ceilLog2(polMan.blockSize) + 1);
    writePktSize.init(ceilLog2(polMan.blockSize) + 1);

    avgRdBWSys.precision(8);
    avgWrBWSys.precision(8);
    avgGap.precision(2);
    accuracyOfDicePredictor.precision(8);
    accuracyOfBypassDcachePredictor.precision(8);

    // Formula stats
    avgRdBWSys = (bytesReadSys) / simSeconds;
    avgWrBWSys = (bytesWrittenSys) / simSeconds;

    avgGap = totGap / (readReqs + writeReqs);

    accuracyOfDicePredictor = numOfTimesDicePredictorCorrect/(numOfTimesDicePredictorCorrect + numOfTimesDicePredictorIncorrect);
    accuracyOfBypassDcachePredictor = numOfTimesBypassDcachePredictorCorrect/(numOfTimesBypassDcachePredictorCorrect + numOfTimesBypassDcachePredictorIncorrect);

}

Port &
PolicyManager::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (SimpleMemobj.py)
    if (if_name == "port") {
        return port;
    } else if (if_name == "loc_req_port") {
        return locReqPort;
    } else if (if_name == "far_req_port") {
        return farReqPort;
    } else {
        // pass it along to our super class
        panic("PORT NAME ERROR !!!!\n");
    }
}

DrainState
PolicyManager::drain()
{
    if (!ORB.empty() || !pktFarMemWrite.empty()) {
        DPRINTF(Drain, "DRAM cache is not drained! Have %d in ORB and %d in "
                "writeback queue.\n", ORB.size(), pktFarMemWrite.size());
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
PolicyManager::serialize(CheckpointOut &cp) const
{
    ScopedCheckpointSection sec(cp, "tagMetadataStore");
    paramOut(cp, "numEntries", tagMetadataStore.size());

    int count = 0;
    for (auto const &entry : tagMetadataStore) {
        ScopedCheckpointSection sec_entry(cp,csprintf("Entry%d", count++));
        if (entry.validLine) {
            paramOut(cp, "validLine", entry.validLine);
            paramOut(cp, "tagDC", entry.tagDC);
            paramOut(cp, "indexDC", entry.indexDC);
            paramOut(cp, "dirtyLine", entry.dirtyLine);
            paramOut(cp, "farMemAddr", entry.farMemAddr);
        } else {
            paramOut(cp, "validLine", entry.validLine);
        }
    }
}

void
PolicyManager::unserialize(CheckpointIn &cp)
{
    ScopedCheckpointSection sec(cp, "tagMetadataStore");
    int num_entries = 0;
    paramIn(cp, "numEntries", num_entries);
    warn_if(num_entries > tagMetadataStore.size(), "Unserializing larger tag "
            "store into a smaller tag store. Stopping when index doesn't fit");
    warn_if(num_entries < tagMetadataStore.size(), "Unserializing smaller "
            "tag store into a larger tag store. Not fully warmed up.");

    for (int i = 0; i < num_entries; i++) {
        ScopedCheckpointSection sec_entry(cp,csprintf("Entry%d", i));

        bool valid = false;
        paramIn(cp, "validLine", valid);
        if (valid) {
            Addr tag = 0;
            Addr index = 0;
            bool dirty = false;
            Addr far_addr = 0;
            paramIn(cp, "tagDC", tag);
            paramIn(cp, "indexDC", index);
            paramIn(cp, "dirtyLine", dirty);
            paramIn(cp, "farMemAddr", far_addr);
            if (index < tagMetadataStore.size()) {
                // Only insert if this entry fits into the current store.
                tagMetadataStore[index].tagDC = tag;
                tagMetadataStore[index].indexDC = index;
                tagMetadataStore[index].validLine = valid;
                tagMetadataStore[index].dirtyLine = dirty;
                tagMetadataStore[index].farMemAddr = far_addr;
            }
        }
    }
}



bool
PolicyManager::RespPortPolManager::recvTimingReq(PacketPtr pkt)
{
    return polMan.recvTimingReq(pkt);
}

} // namespace memory
} // namespace gem5
