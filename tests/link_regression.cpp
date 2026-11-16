// Compile the real UART implementation against small hardware/RTOS adapters.
#include <iostream>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <cmath>   // 先于 namespace 包裹 include，避免 macro_engine.cpp 的 <math.h> 在
                   // macro_fixture::std 内展开导致 std::sqrt(double) 重载缺失/歧义。
bool mock_stop_empty = false;
bool mock_stop_on_delay = false;
std::vector<uint8_t> mock_rx;
size_t mock_rx_pos = 0;
std::vector<std::vector<uint8_t>> mock_tx;
unsigned mock_tx_waits = 0;
unsigned mock_fail_drains = 0;
void (*mock_on_drain)() = nullptr;
std::vector<char> mock_tx_order;
#include "../components/esp_link/esp_link.cpp"
#include "../components/macro_engine/include/macro_engine.h"
// Namespace only separates private file-scope names; the engine still calls the
// real global esp_link APIs and uses the same RTOS queue adapters.
namespace macro_fixture {
#include "../components/macro_engine/macro_engine.cpp"
}
static void macro_tick() {
    mock_stop_on_delay=true;
    try { macro_fixture::macro_task(nullptr); } catch(const StopTask&) {}
    mock_stop_on_delay=false;
}

static void check(bool ok, const char* reason) {
    if(!ok) throw std::runtime_error(reason);
}
static std::vector<uint8_t> packet(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> v{0xAA,0x55,static_cast<uint8_t>(payload.size())};
    v.insert(v.end(),payload.begin(),payload.end());
    uint16_t crc=0xffff;
    for(uint8_t b:v) {
        crc ^= static_cast<uint16_t>(b)<<8;
        for(int i=0;i<8;++i) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
    }
    v.push_back(crc&0xff); v.push_back(crc>>8); return v;
}
static void receive(const std::vector<uint8_t>& bytes) {
    mock_rx=bytes; mock_rx_pos=0;
    try { esp_link_rx_task(nullptr); } catch(const StopTask&) {}
}
int main() {
    try {
        esp_link_queues_init();
        std::vector<uint8_t> payload(sizeof(esp_link_sensor_t));
        for(size_t i=0;i<payload.size();++i) payload[i]=static_cast<uint8_t>(i);
        receive(packet(payload));
        esp_link_snapshot_t s{};
        check(xQueuePeek(esp_link_sensor_queue,&s,0)==pdTRUE,"valid 112-byte telemetry frame was rejected");
        check(std::memcmp(&s.sensor,payload.data(),payload.size())==0,"telemetry payload corrupted");
        std::cout << "PASS valid 112-byte frame and header-inclusive CRC\n";
        for(unsigned i=0;i<8;++i)
            check(esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"queue filling failed");
        check(esp_link_send_command(ESP_CMD_STOP_ALL,nullptr,0),"STOP_ALL lost when ordinary command queue is full");
        std::cout << "PASS STOP_ALL cannot be dropped by a full queue\n";
        const uint32_t stopped_generation = esp_link_command_generation();
        check(esp_link_commands_inhibited(),"STOP did not latch");
        check(!esp_link_resume_commands(stopped_generation),"resumed before STOP was transmitted");
        check(!esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"normal command accepted during stop");
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(mock_tx.size()==1 && mock_tx[0]==packet({ESP_CMD_STOP_ALL}),"STOP did not replace stale queued commands");
        check(mock_tx_waits==mock_tx.size(),"TX did not wait for wire completion");
        check(esp_link_resume_commands(stopped_generation),"fresh resume failed after transmitted stop");
        check(!esp_link_send_command(ESP_CMD_PID,nullptr,8),"null parameter accepted");
        check(esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"fresh command rejected");
        esp_link_request_stop();
        check(!esp_link_resume_commands(stopped_generation),"stale generation resumed motion");
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(esp_link_resume_commands(esp_link_command_generation()),"current generation failed to resume");
        check(!esp_link_send_command_for_generation(ESP_CMD_LED_TOGGLE,nullptr,0,stopped_generation),"stale producer enqueued after a newer resume");
        check(mock_tx.size()==2 && mock_tx[1]==packet({ESP_CMD_STOP_ALL}),"old command crossed newer stop barrier");
        std::cout << "PASS stop latch, stale generations, invalid pointer and UART drain\n";
        xQueueReset(esp_link_sensor_queue);
        auto bad_crc=packet(payload); bad_crc.back()^=0x80;
        receive(bad_crc);
        check(xQueuePeek(esp_link_sensor_queue,&s,0)==pdFALSE,"bad CRC accepted");
        for(unsigned len=0;len<256;++len) {
            if(len==payload.size()) continue;
            std::vector<uint8_t> bytes{0xAA,0x55,static_cast<uint8_t>(len)};
            bytes.insert(bytes.end(),600,0);
            auto valid=packet(payload); bytes.insert(bytes.end(),valid.begin(),valid.end());
            receive(bytes);
            check(xQueueReceive(esp_link_sensor_queue,&s,0)==pdTRUE,"failed to recover after invalid length");
            check(std::memcmp(&s.sensor,payload.data(),payload.size())==0,"invalid length damaged subsequent frame");
        }
        std::cout << "PASS all 255 invalid lengths including zero, recovery and CRC rejection\n";
        std::vector<uint8_t> embedded(112,0);
        embedded[64]=0xAA; embedded[65]=0x55; embedded[66]=112;
        auto dropped=packet(embedded); dropped.pop_back();
        auto complete=packet(embedded);
        for(int i=0;i<20;++i) dropped.insert(dropped.end(),complete.begin(),complete.end());
        receive(dropped);
        check(xQueuePeek(esp_link_sensor_queue,&s,0)==pdTRUE,"lost UART byte permanently desynchronized embedded-header stream");
        check(std::memcmp(&s.sensor,embedded.data(),embedded.size())==0,"recovered payload mismatch");
        std::cout << "PASS dropped CRC byte with repeated embedded frame headers\n";
        mock_tx.clear(); mock_tx_order.clear();
        check(esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"fresh command failed");
        check(esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"second command failed");
        mock_on_drain=+[] { esp_link_request_stop(); };
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(mock_tx.size()==2 && mock_tx[0]==packet({ESP_CMD_LED_TOGGLE}) && mock_tx[1]==packet({ESP_CMD_STOP_ALL}),"in-flight stop ordering failed");
        check(mock_tx_order==std::vector<char>({'W','D','I','W','D','I'}),"write/drain/idle ordering failed");
        mock_tx.clear(); mock_tx_order.clear();
        esp_link_request_stop();
        mock_on_drain=+[] { esp_link_request_stop(); };
        mock_fail_drains=1;
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(mock_tx.size()==2 && mock_tx[0]==packet({ESP_CMD_STOP_ALL}) && mock_tx[1]==packet({ESP_CMD_STOP_ALL}),"failed stop/newer stop not retried");
        check(esp_link_commands_inhibited(),"successful retry silently rearmed");
        check(esp_link_resume_commands(esp_link_command_generation()),"retry completion not acknowledged");
        std::cout << "PASS in-flight stop, repeated stop, TX failure retry and drain/idle order\n";
        macro_fixture::macro_queues_init();
        auto *program=static_cast<macro_prog_t*>(calloc(1,sizeof(macro_prog_t)));
        program->id=42; program->count=1;
        program->steps[0].type=MACRO_STEP_ACT;
        program->steps[0].u.act.cmd_id=ESP_CMD_LED_TOGGLE;
        macro_ctrl_t start={}; start.type=MACRO_CTRL_START; start.prog=program;
        start.generation=esp_link_command_generation();
        xQueueSend(macro_fixture::macro_ctrl_queue,&start,0);
        mock_tx.clear();
        macro_tick(); // Deliberately do NOT schedule UART TX until ACT is queued.
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(!mock_tx.empty() && mock_tx[0]==packet({ESP_CMD_LED_TOGGLE}),"macro completion discarded its final unsent ACT");
        macro_tick();
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(mock_tx.size()==2 && mock_tx[1]==packet({ESP_CMD_STOP_ALL}),"normal macro completion did not stop after final ACT");
        macro_progress_t progress={};
        check(xQueuePeek(macro_fixture::macro_progress_queue,&progress,0)==pdTRUE && progress.id==42 && progress.state==MACRO_STATE_DONE,"completed macro progress mismatch");
        std::cout << "PASS real macro + UART delayed-TX completion ordering\n";
        const uint32_t current=esp_link_command_generation();
        check(esp_link_resume_commands(current),"could not resume after macro stop");
        check(esp_link_commands_drained(current),"empty active generation not drained");
        for(int i=0;i<8;++i) check(esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"fill for completion accounting failed");
        check(!esp_link_send_command(ESP_CMD_LED_TOGGLE,nullptr,0),"full queue unexpectedly accepted command");
        check(!esp_link_commands_drained(current),"queued commands reported as physically drained");
        mock_on_drain=+[] { check(!esp_link_commands_drained(esp_link_command_generation()),"drained before UART completion returned"); };
        mock_stop_empty=true;
        try { esp_link_tx_task(nullptr); } catch(const StopTask&) {}
        mock_stop_empty=false;
        check(esp_link_commands_drained(current),"queue-full rollback leaked pending TX count");
        check(esp_link_crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29B1,"CRC reference vector mismatch");
        std::cout << "PASS TX completion accounting, full-queue rollback and CRC reference vector\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n'; return 1; }
}
