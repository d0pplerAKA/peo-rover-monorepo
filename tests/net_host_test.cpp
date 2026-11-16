// Execute the production translation units; only RTOS/HAL are substituted.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <cJSON.h>
#include "esp_link.h"
#include "c3_link.h"
#include "camera_service.h"
#include "freertos/task.h"
QueueHandle_t esp_link_cmd_queue=nullptr, esp_link_sensor_queue=nullptr, xbox_state_queue=nullptr;
static bool inhibited=false;
static bool fatal_fault=false;  // 本套件的安全边界替身；不等同于真实 esp_link 锁实现。
static bool allow_resume=true;
static bool commands_drained=true;
static std::function<void()> before_enqueue;
static std::function<void()> after_drain_check;
static uint32_t generation=1;
static std::vector<uint8_t> commands;
void esp_link_request_stop() { inhibited=true; ++generation; commands.push_back(ESP_CMD_STOP_ALL); }
void esp_link_fatal_fault() { fatal_fault=true; esp_link_request_stop(); }
bool esp_link_commands_inhibited() {return inhibited;}
uint32_t esp_link_command_generation() {return generation;}
bool esp_link_commands_drained(uint32_t g) {bool drained=g==generation && !inhibited && commands_drained;if(after_drain_check){auto hook=after_drain_check;after_drain_check=nullptr;hook();}return drained;}
bool esp_link_resume_commands(uint32_t g) {if(g!=generation||!allow_resume||fatal_fault)return false;inhibited=false;return true;}
bool esp_link_send_command(uint8_t id,const uint8_t*,uint8_t) {if(before_enqueue){auto hook=before_enqueue;before_enqueue=nullptr;hook();}if(id==ESP_CMD_STOP_ALL){esp_link_request_stop();return true;}if(inhibited)return false;commands.push_back(id);return true;}
bool esp_link_send_command_for_generation(uint8_t id,const uint8_t* p,uint8_t len,uint32_t g) {if(before_enqueue){auto hook=before_enqueue;before_enqueue=nullptr;hook();}return g==generation && esp_link_send_command(id,p,len);}
// This suite has no camera service; provide only the parser's required link symbols.
bool camera_resolution_from_name(const char*,uint8_t*) {return false;}
bool camera_request_resolution(uint8_t) {return false;}
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <fcntl.h>
#include <lwip/sockets.h>
// Fault only application allocations; cJSON uses its public allocator hooks below.
static int fail_allocation_after=-1;
static bool fail_allocation(){if(fail_allocation_after<0)return false;return fail_allocation_after--==0;}
static void* app_malloc(size_t n){return fail_allocation()?nullptr:malloc(n);}
static void* app_realloc(void* p,size_t n){return fail_allocation()?nullptr:realloc(p,n);}
static char* app_strdup(const char* text){if(fail_allocation())return nullptr;size_t n=strlen(text)+1;char* p=(char*)malloc(n);if(p)memcpy(p,text,n);return p;}
#define malloc app_malloc
#define realloc app_realloc
#define strdup app_strdup
#define TAG macro_tag
#include "../components/macro_engine/macro_engine.cpp"
#undef TAG
#define TAG parser_tag
#include "../components/cmd_parser/cmd_parser.cpp"
#undef TAG
#define TAG telemetry_tag
#include "../components/telemetry/telemetry.cpp"
#undef TAG
#define TAG net_tag
#include "../components/esp_net/esp_net.cpp"
#undef TAG
#undef malloc
#undef realloc
#undef strdup
#define CHECK(v) do { if(!(v)) throw std::runtime_error(#v); } while(0)
static void newline_test() {
  // Send an actual production ACK through the production broadcast loop.
  esp_net_queues_init(); macro_queues_init();
  int cycles=0;
  sockaddr_in peer{};peer.sin_family=AF_INET;peer.sin_addr.s_addr=htonl(0xc0a80402);host_accept.push_back(peer);
  select_hook=[&](fd_set* r,fd_set* w){if(++cycles>2)throw TaskDone{}; FD_ZERO(r);if(w)FD_ZERO(w);if(cycles==1)FD_SET(10,r);if(cycles==2){send_ack("LED_TOGGLE",0);if(w)FD_SET(12,w);}};
  try {esp_net_task(nullptr);} catch(TaskDone&){}
  CHECK(host_sent[12]=="{\"ack\":\"LED_TOGGLE\",\"status\":0}\n");
}
static void receive_test(bool oversized) {
  esp_net_queues_init(); macro_queues_init();
  std::string input;
  if(oversized) input=std::string(16385,'x')+"\n{\"cmd\":\"LED_TOGGLE\"}\n";
  else {input="{\"type\":\"macro\",\"id\":7,\"steps\":[";for(int i=0;i<64;++i){if(i)input+=",";input+="{\"cmd\":\"LED_TOGGLE\",\"duration_ms\":0}";}input+="]}\n";}
  host_received[12]=input;
  sockaddr_in peer{};host_accept.push_back(peer);
  int cycles=0;
  select_hook=[&](fd_set* r,fd_set* w){if(++cycles>100)throw TaskDone{};FD_ZERO(r);if(w)FD_ZERO(w);if(cycles==1)FD_SET(10,r);else if(!host_received[12].empty())FD_SET(12,r);else throw TaskDone{};};
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  if(oversized) {CHECK(esp_net_rx_queue->data.empty());CHECK(inhibited);}
  else {
    CHECK(esp_net_rx_queue->data.size()==1);
    esp_net_rx_message_t message{};memcpy(&message,esp_net_rx_queue->data.front().data(),sizeof(message));
    char* line=message.line;
    CHECK(std::string(line)+"\n"==input);
    cJSON* root=cJSON_Parse(line);CHECK(root!=nullptr);CHECK(cJSON_GetArraySize(cJSON_GetObjectItem(root,"steps"))==64);cJSON_Delete(root);
    receive_budget=1;try{cmd_parser_task(nullptr);}catch(TaskDone&){}receive_budget=-1;
    macro_ctrl_t control{};CHECK(xQueueReceive(macro_ctrl_queue,&control,0));CHECK(control.prog->count==64);CHECK(control.generation==generation);free(control.prog);
  }
}
static void send_test(bool slow) {
  esp_net_queues_init();macro_queues_init();
  sockaddr_in peer{};peer.sin_family=AF_INET;host_accept.push_back(peer);
  host_send_limit[12]=slow?0:3;
  const std::string expected="{\"n\":1}\n{\"n\":2}\n";
  const uint32_t epoch=generation;
  int cycles=0,pings_sent=0,pings_parsed=0;
  uint32_t last_ping_ms=0,stall_started_ms=0;
  bool reconnect_requested=false,healthy_sequence_queued=false;
  auto enqueue_sequence=[&](){
    char* out=strdup("{\"n\":1}");CHECK(out);CHECK(xQueueSend(esp_net_tx_queue,&out,0));
    out=strdup("{\"n\":2}");CHECK(out);CHECK(xQueueSend(esp_net_tx_queue,&out,0));
  };
  select_hook=[&](fd_set* r,fd_set* w){
    CHECK(++cycles<=300);  /* 保留超过 TX 超时的上界，完成条件由真实发送/断开状态决定。 */
    host_ticks+=20;FD_ZERO(r);if(w)FD_ZERO(w);
    /* 模拟解析器调度：只处理上一轮真实 recv 入队的消息，不直接改活动时间。 */
    while(!esp_net_rx_queue->data.empty()){
      esp_net_rx_message_t message{};CHECK(xQueuePeek(esp_net_rx_queue,&message,0));
      const bool ping=strstr(message.line,"PING")!=nullptr;
      receive_budget=1;try{cmd_parser_task(nullptr);}catch(TaskDone&){}receive_budget=-1;
      CHECK(s_client_activity[0].last_cmd_ms==message.received_ms);
      if(ping)++pings_parsed;
    }
    if(cycles==1){FD_SET(10,r);return;}
    if(!slow){
      CHECK(esp_net_has_client());CHECK(host_closed.empty());
      if(cycles==2)enqueue_sequence();
      if(host_sent[12]==expected){
        CHECK(host_ticks<PEO_NET_HEARTBEAT_TIMEOUT_MS);CHECK(generation==epoch);
        throw TaskDone{};
      }
      return;
    }
    if(std::find(host_closed.begin(),host_closed.end(),12)!=host_closed.end()){
      CHECK(std::count(host_closed.begin(),host_closed.end(),12)==1);
      CHECK(host_sent[12].empty());CHECK(generation==epoch+1);
      CHECK(std::count(commands.begin(),commands.end(),ESP_CMD_STOP_ALL)==1);
      if(!reconnect_requested){
        CHECK(!esp_net_has_client());CHECK(host_ticks-stall_started_ms>=NET_TX_TIMEOUT_MS);
        CHECK(pings_parsed>0&&pings_parsed==pings_sent);
        host_accept.push_back(peer);FD_SET(10,r);reconnect_requested=true;
      }else{
        CHECK(esp_net_has_client());CHECK(s_client_activity[0].fd==13);
        if(!healthy_sequence_queued){enqueue_sequence();healthy_sequence_queued=true;}
        else if(host_sent[13]==expected){
          CHECK(std::find(host_closed.begin(),host_closed.end(),13)==host_closed.end());
          throw TaskDone{};
        }
      }
      return;
    }
    CHECK(esp_net_has_client());CHECK(s_client_activity[0].fd==12);
    if(cycles==2){
      /* 零速 MOTOR 走真实接收/解析，进入控制会话心跳监督；LED 命令不会启用监督。 */
      host_received[12]="{\"cmd\":\"MOTOR\",\"motor\":0,\"speed\":0}\n";FD_SET(12,r);
    }
    if(cycles==3){CHECK(s_client_activity[0].has_commanded);enqueue_sequence();stall_started_ms=host_ticks;}
    if(host_ticks-last_ping_ms>=500){
      CHECK(host_received[12].empty());
      host_received[12]="{\"cmd\":\"PING\",\"t\":"+std::to_string(host_ticks)+"}\n";
      FD_SET(12,r);last_ping_ms=host_ticks;++pings_sent;
    }
  };
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  if(slow){
    CHECK(reconnect_requested&&healthy_sequence_queued);CHECK(pings_parsed>0);
    CHECK(host_sent[13]==expected);CHECK(inhibited);
  }else CHECK(host_sent[12]==expected);
}
static void video_enabled_test() {
  CHECK(!esp_net_video_enabled());
  esp_net_video_set_enabled(true);CHECK(esp_net_video_enabled());
  esp_net_video_set_enabled(true);CHECK(esp_net_video_enabled());
  esp_net_video_set_enabled(false);CHECK(!esp_net_video_enabled());
}
static void video_disconnect_test() {
  esp_net_queues_init();macro_queues_init();
  sockaddr_in peer{};peer.sin_family=AF_INET;host_accept.push_back(peer);
  int cycles=0;uint8_t jpeg[]={'J'};camera_fb_t fb{jpeg,sizeof(jpeg)};
  select_hook=[&](fd_set* r,fd_set* w){
    FD_ZERO(r);if(w)FD_ZERO(w);CHECK(++cycles<=5);
    if(cycles==1){FD_SET(10,r);}
    else if(cycles==2){
      CHECK(esp_net_has_client());CHECK(!esp_net_video_enabled());
      esp_net_video_set_enabled(true);esp_net_video_set_frame_owner(&fb);
      CHECK(esp_net_send_video(fb.buf,fb.len));CHECK(host_udp.empty());
      FD_SET(12,r);  /* 本轮真实 recv 返回 EOF；先 drop/flush，再进入视频消费分支。 */
    }else if(cycles==3){
      CHECK(!esp_net_has_client());CHECK(!esp_net_video_enabled());
      CHECK(std::count(host_closed.begin(),host_closed.end(),12)==1);
      CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&fb);
      CHECK(!video_pending());CHECK(host_udp.empty());
      /* 单客户端模式必须先回收旧槽，再重连；重连不能自动恢复视频开关。 */
      host_accept.push_back(peer);FD_SET(10,r);
    }else if(cycles==4){
      CHECK(esp_net_has_client());CHECK(s_client_activity[0].fd==13);
      CHECK(!esp_net_video_enabled());CHECK(host_returned_frames.size()==1);
      FD_SET(13,r);
    }else{
      CHECK(!esp_net_has_client());CHECK(!esp_net_video_enabled());
      CHECK(std::count(host_closed.begin(),host_closed.end(),13)==1);
      CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&fb);
      CHECK(host_udp.empty());throw TaskDone{};
    }
  };
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  CHECK(cycles==5);
}
static void udp_test() {
  esp_net_queues_init();macro_queues_init();
  sockaddr_in peer{};peer.sin_family=AF_INET;peer.sin_addr.s_addr=htonl(0xc0a80402);host_accept.push_back(peer);
  constexpr size_t frame_count=4;
  const size_t payload_size=PEO_NET_VIDEO_MAX_PKT+1;
  const size_t packets_per_frame=(payload_size+PEO_NET_VIDEO_MAX_PKT-1)/PEO_NET_VIDEO_MAX_PKT;
  std::string jpeg[frame_count];camera_fb_t frames[frame_count]{};
  for(size_t i=0;i<frame_count;++i){jpeg[i]=std::string(payload_size,(char)('A'+i));frames[i]={(uint8_t*)jpeg[i].data(),jpeg[i].size()};}
  size_t cycles=0;
  select_hook=[&](fd_set* r,fd_set* w){
    FD_ZERO(r);if(w)FD_ZERO(w);CHECK(++cycles<=frame_count+2);
    if(cycles==1){FD_SET(10,r);return;}
    const size_t completed=cycles-2;
    /* 每轮先确认上一帧已发送并归还，再提交下一帧，不把覆盖队列当作积压队列。 */
    CHECK(host_udp.size()==completed*packets_per_frame);CHECK(host_returned_frames.size()==completed);
    if(completed==frame_count){throw TaskDone{};}
    if(completed)CHECK(host_returned_frames[completed-1]==&frames[completed-1]);
    esp_net_video_set_enabled(true);esp_net_video_set_frame_owner(&frames[completed]);
    CHECK(esp_net_send_video(frames[completed].buf,frames[completed].len));
    CHECK(host_udp.size()==completed*packets_per_frame);CHECK(host_returned_frames.size()==completed);
    CHECK(s_video_pending.owner==&frames[completed]&&s_video_pending.buf==frames[completed].buf);
    CHECK(host_task_notification.pending);
    /* 返回生产网络循环，由本轮末尾的 send_pending_video 发送整帧。 */
  };
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  CHECK(host_udp.size()==frame_count*packets_per_frame);CHECK(host_returned_frames.size()==frame_count);
  uint16_t previous_fid=0;
  for(size_t frame=0;frame<frame_count;++frame){
    CHECK(host_returned_frames[frame]==&frames[frame]);uint16_t fid=0;
    for(size_t part=0;part<packets_per_frame;++part){
      const auto& packet=host_udp[frame*packets_per_frame+part];const std::string& bytes=packet.second;
      const size_t offset=part*PEO_NET_VIDEO_MAX_PKT;
      const size_t chunk=std::min((size_t)PEO_NET_VIDEO_MAX_PKT,payload_size-offset);
      CHECK(packet.first.sin_port==htons(8889));CHECK(packet.first.sin_addr.s_addr==peer.sin_addr.s_addr);
      CHECK(bytes.size()==8+chunk);CHECK((uint8_t)bytes[0]==0xca&&(uint8_t)bytes[1]==0xfe);
      const uint16_t packet_fid=((uint16_t)(uint8_t)bytes[2]<<8)|(uint8_t)bytes[3];
      if(part==0)fid=packet_fid;else CHECK(packet_fid==fid);
      const size_t packet_part=((size_t)(uint8_t)bytes[4]<<8)|(uint8_t)bytes[5];
      const size_t packet_total=((size_t)(uint8_t)bytes[6]<<8)|(uint8_t)bytes[7];
      CHECK(packet_part==part);CHECK(packet_total==packets_per_frame);
      CHECK(bytes.substr(8)==jpeg[frame].substr(offset,chunk));
    }
    if(frame)CHECK(fid==(uint16_t)(previous_fid+1));
    previous_fid=fid;
  }
}
/* 无 RTOS 调度器时直接驱动同一消费函数；udp/disconnect 用例另走完整网络循环。 */
static void video_ownership_test(const std::string& mode) {
  esp_net_queues_init();s_udp_fd=11;s_net_task=xTaskGetCurrentTaskHandle();
  s_video_peers[0].sin_family=AF_INET;s_video_peers[0].sin_port=htons(8889);
  esp_net_video_set_enabled(true);
  uint8_t jpeg_a[]={'A'},jpeg_b[]={'B'};
  camera_fb_t first{jpeg_a,sizeof(jpeg_a)},second{jpeg_b,sizeof(jpeg_b)};
  if(mode=="reject"){
    esp_net_video_set_enabled(false);esp_net_video_set_frame_owner(&first);
    CHECK(!esp_net_send_video(first.buf,first.len));CHECK(host_returned_frames.empty());
    esp_net_video_set_enabled(true);s_video_peers[0]={};esp_net_video_set_frame_owner(&first);
    CHECK(!esp_net_send_video(first.buf,first.len));CHECK(host_returned_frames.empty());
    esp_camera_fb_return(&first);CHECK(host_returned_frames.size()==1);return;
  }
  esp_net_video_set_frame_owner(&first);CHECK(esp_net_send_video(first.buf,first.len));
  CHECK(host_udp.empty());CHECK(host_returned_frames.empty());
  if(mode=="overwrite"){
    esp_net_video_set_frame_owner(&second);CHECK(esp_net_send_video(second.buf,second.len));
    CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&first);
    CHECK(s_video_pending.owner==&second&&s_video_pending.buf==second.buf);
    send_pending_video();CHECK(host_returned_frames.size()==2&&host_returned_frames[1]==&second);
    CHECK(host_udp.size()==1&&host_udp[0].second[8]=='B');
  }else if(mode=="flush"){
    esp_net_video_flush();esp_net_video_flush();
    CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&first);
    CHECK(!video_pending());CHECK(host_udp.empty());
  }else if(mode=="nopeer"){
    s_video_peers[0]={};send_pending_video();
    CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&first);CHECK(host_udp.empty());
  }else if(mode=="sendfail"){
    host_sendto_errno=EAGAIN;send_pending_video();host_sendto_errno=0;
    CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&first);CHECK(host_udp.empty());
  }else if(mode=="inflight"){
    host_sendto_hook=[&](){
      host_sendto_hook=nullptr;CHECK(s_video_send_mutex->locked);CHECK(host_returned_frames.empty());
      esp_net_video_set_frame_owner(&second);CHECK(esp_net_send_video(second.buf,second.len));
      CHECK(s_video_pending.owner==&second);
    };
    send_pending_video();CHECK(host_returned_frames.size()==1&&host_returned_frames[0]==&first);
    CHECK(s_video_pending.owner==&second);send_pending_video();
    CHECK(host_returned_frames.size()==2&&host_returned_frames[1]==&second);CHECK(host_udp.size()==2);
  }else CHECK(false);
}
static void macro_test(const std::string& name) {
  macro_queues_init();
  auto* prog=(macro_prog_t*)calloc(1,sizeof(macro_prog_t));prog->id=77;prog->count=2;
  prog->steps[1].type=MACRO_STEP_ACT;prog->steps[1].u.act.cmd_id=ESP_CMD_LED_TOGGLE;
  if(name=="progress") {prog->count=1;prog->steps[0]=prog->steps[1];}
  else {prog->steps[0].type=MACRO_STEP_GOTO;prog->steps[0].u.got.target=name=="goto_negative"?-1:64;}
  macro_ctrl_t c={};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=prog;
  xQueueSend(macro_ctrl_queue,&c,0);
  delay_hook=[](){throw TaskDone{};};
  try{macro_task(nullptr);}catch(TaskDone&){}
  macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));
  CHECK(p.state==MACRO_STATE_DONE);CHECK(p.id==77);CHECK(p.total==(name=="progress"?1:2));CHECK(s_cur==nullptr);
}
static void completion_drain_test(const std::string& ending) {
  macro_queues_init();commands_drained=false;uint32_t epoch=generation;
  auto* prog=(macro_prog_t*)calloc(1,sizeof(macro_prog_t));prog->id=78;prog->count=1;
  prog->steps[0].type=MACRO_STEP_ACT;prog->steps[0].u.act.cmd_id=ESP_CMD_LED_TOGGLE;
  if(ending!="act"){
    prog->count=2;prog->steps[1].type=MACRO_STEP_GOTO;
    prog->steps[1].u.got.target=ending=="goto_negative"?-1:64;
  }
  uint16_t total=prog->count;
  macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=epoch;c.prog=prog;CHECK(xQueueSend(macro_ctrl_queue,&c,0));
  int cycles=0;delay_hook=[&](){
    if(++cycles>100)throw TaskDone{};
    macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));
    CHECK(p.state==MACRO_STATE_RUNNING);CHECK(p.id==78);CHECK(p.total==total);
    CHECK(s_cur!=nullptr);CHECK(generation==epoch);CHECK(commands.size()==1&&commands[0]==ESP_CMD_LED_TOGGLE);
    if(cycles==100)commands_drained=true;
  };
  try{macro_task(nullptr);}catch(TaskDone&){}
  macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));
  CHECK(p.state==MACRO_STATE_DONE);CHECK(p.id==78);CHECK(p.total==total);CHECK(s_cur==nullptr);
  CHECK(commands.size()==2&&commands[1]==ESP_CMD_STOP_ALL);CHECK(generation==epoch+1);
}
/* 超时回收后多轮保持一次 STOP，并验证同一槽位立即接受新连接。 */
static void heartbeat_timeout_test(bool initially_latched) {
  esp_net_queues_init();macro_queues_init();CHECK(!esp_net_has_client());
  sockaddr_in peer{};peer.sin_family=AF_INET;host_accept.push_back(peer);
  const uint32_t epoch=generation;host_ticks=100;
  const int drop_cycle=initially_latched?3:2;int cycles=0;
  select_hook=[&](fd_set* r,fd_set* w){
    FD_ZERO(r);if(w)FD_ZERO(w);++cycles;
    if(cycles==1){CHECK(!esp_net_has_client());FD_SET(10,r);}
    else if(cycles==2){
      CHECK(esp_net_has_client());CHECK(s_client_activity[0].fd==12);
      esp_net_note_client_activity(0,100,true);
      s_client_activity[0].stop_latched=initially_latched;
      esp_net_video_set_enabled(true);host_ticks=100+PEO_NET_HEARTBEAT_TIMEOUT_MS+1;
    }else if(initially_latched&&cycles==3){
      CHECK(commands.empty());CHECK(host_closed.empty());CHECK(esp_net_has_client());
      CHECK(s_client_activity[0].stop_latched);
      esp_net_note_client_activity(0,host_ticks,false);
      CHECK(!s_client_activity[0].stop_latched);CHECK(s_client_activity[0].has_commanded);
      host_ticks+=PEO_NET_HEARTBEAT_TIMEOUT_MS+1;
    }else{
      CHECK(commands.size()==1&&commands[0]==ESP_CMD_STOP_ALL);CHECK(generation==epoch+1);
      CHECK(std::count(host_closed.begin(),host_closed.end(),12)==1);
      if(cycles==drop_cycle+4){
        CHECK(esp_net_has_client());CHECK(s_client_activity[0].fd==13);
        CHECK(!s_client_activity[0].has_commanded&&!s_client_activity[0].stop_latched);
        throw TaskDone{};
      }
      CHECK(!esp_net_has_client());CHECK(s_client_activity[0].fd==-1);
      CHECK(!s_client_activity[0].has_commanded&&!s_client_activity[0].stop_latched);
      CHECK(s_video_peers[0].sin_family==0);CHECK(!esp_net_video_enabled());
      host_ticks+=20;
      if(cycles==drop_cycle+3){host_accept.push_back(peer);FD_SET(10,r);}
    }
  };
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  CHECK(cycles==drop_cycle+4);CHECK(commands.size()==1);
}
static void activity_latch_reset_test() {
  CHECK(!esp_net_has_client());s_client_activity[0].stop_latched=true;
  esp_net_note_client_activity(0,123,true);
  CHECK(s_client_activity[0].last_cmd_ms==123&&!s_client_activity[0].stop_latched);
  CHECK(s_client_activity[0].has_commanded);CHECK(!esp_net_has_client());
  s_client_activity[0].stop_latched=true;esp_net_note_client_activity(0,124,false);
  CHECK(s_client_activity[0].last_cmd_ms==124&&!s_client_activity[0].stop_latched);
  CHECK(s_client_activity[0].has_commanded);CHECK(!esp_net_has_client());
}
static void snapshot_test() {
  esp_net_queues_init();macro_queues_init();
  s_client_activity[0].fd=12;  /* 遥测用例模拟在线客户端，使用生产查询函数 */
  esp_link_sensor_queue=xQueueCreate(1,sizeof(esp_link_snapshot_t));xbox_state_queue=xQueueCreate(1,sizeof(xbox_state_t));
  esp_link_snapshot_t s{};s.sensor.sys_tick_ms=123;s.received_ms=xTaskGetTickCount();s.seq=1;xQueueOverwrite(esp_link_sensor_queue,&s);
  receive_budget=2;int delays=0;delay_hook=[&](){if(++delays==2)throw TaskDone{};};
  try{telemetry_task(nullptr);}catch(TaskDone&){}
  CHECK(esp_net_tx_queue->data.size()==1);
  receive_budget=-1;
  CHECK(xQueuePeek(esp_link_sensor_queue,&s,0)==pdTRUE);CHECK(s.sensor.sys_tick_ms==123);
}
static void missing_xbox_queue_test() {
  esp_net_queues_init();macro_queues_init();
  s_client_activity[0].fd=12;  /* 保留原用例的广播前提 */
  esp_link_sensor_queue=xQueueCreate(1,sizeof(esp_link_snapshot_t));
  esp_link_snapshot_t sensor{};sensor.sensor.sys_tick_ms=456;sensor.received_ms=xTaskGetTickCount();sensor.seq=1;xQueueOverwrite(esp_link_sensor_queue,&sensor);
  CHECK(xbox_state_queue==nullptr);
  int cycles=0;delay_hook=[&](){if(++cycles==2)throw TaskDone{};};
  try{telemetry_task(nullptr);}catch(TaskDone&){}
  char* out=nullptr;CHECK(xQueueReceive(esp_net_tx_queue,&out,0));
  cJSON* root=cJSON_Parse(out);CHECK(root);free(out);
  CHECK(cJSON_GetObjectItem(root,"tick")->valuedouble==456);
  CHECK(cJSON_GetObjectItem(root,"gamepad")==nullptr);cJSON_Delete(root);
  CHECK(xQueuePeek(esp_link_sensor_queue,&sensor,0));CHECK(sensor.sensor.sys_tick_ms==456);
}
static int json_fail_at=-1,json_allocations=0,json_live=0;
static void* json_malloc(size_t n){if(json_allocations++==json_fail_at)return nullptr;void* p=malloc(n);if(p)++json_live;return p;}
static void json_free(void* p){if(p)--json_live;free(p);}
static void telemetry_client_gate_test(bool reconnect) {
  esp_net_queues_init();macro_queues_init();CHECK(!esp_net_has_client());
  esp_link_sensor_queue=xQueueCreate(1,sizeof(esp_link_snapshot_t));
  xbox_state_queue=xQueueCreate(1,sizeof(xbox_state_t));
  esp_link_snapshot_t sensor{};sensor.sensor.sys_tick_ms=789;sensor.seq=7;
  xbox_state_t xbox{};xbox.connected=1;
  CHECK(xQueueOverwrite(esp_link_sensor_queue,&sensor));CHECK(xQueueOverwrite(xbox_state_queue,&xbox));
  json_fail_at=-1;json_allocations=0;json_live=0;
  cJSON_Hooks hooks={json_malloc,json_free};cJSON_InitHooks(&hooks);int delays=0;
  delay_hook=[&](){
    ++delays;
    if(delays==3){
      CHECK(json_allocations==0&&esp_net_tx_queue->data.empty());
      if(reconnect)s_client_activity[0].fd=12;
    }
    if(delays==4)throw TaskDone{};
  };
  try{telemetry_task(nullptr);}catch(TaskDone&){}
  delay_hook=nullptr;
  if(reconnect){
    CHECK(esp_net_has_client());CHECK(json_allocations>0);CHECK(esp_net_tx_queue->data.size()==1);
    char* out=nullptr;CHECK(xQueueReceive(esp_net_tx_queue,&out,0));json_free(out);
  }else{
    CHECK(!esp_net_has_client());CHECK(json_allocations==0&&esp_net_tx_queue->data.empty());
  }
  CHECK(json_live==0);cJSON_InitHooks(nullptr);
  CHECK(xQueuePeek(esp_link_sensor_queue,&sensor,0));CHECK(sensor.sensor.sys_tick_ms==789&&sensor.seq==7);
  CHECK(xQueuePeek(xbox_state_queue,&xbox,0));CHECK(xbox.connected==1);
}
static void oom_test() {
  macro_queues_init();esp_link_sensor_t s{};xbox_state_t xb{};xb.buttons=0xffff;
  macro_progress_t progress{7,1,64,MACRO_STATE_RUNNING};xQueueOverwrite(macro_progress_queue,&progress);
  cJSON_Hooks hooks={json_malloc,json_free};cJSON_InitHooks(&hooks);bool reached_success=false;
  for(int fault=0;fault<1000;++fault){json_fail_at=fault;json_allocations=0;char* out=build_json(&s,&xb,true,false);bool hit=json_allocations>fault;if(out)json_free(out);CHECK(json_live==0);if(hit)CHECK(out==nullptr);else{reached_success=true;break;}}
  CHECK(reached_success);cJSON_InitHooks(nullptr);
}
static macro_prog_t* moving_program() {
  auto* p=(macro_prog_t*)calloc(1,sizeof(macro_prog_t));p->id=90;p->count=2;
  for(auto& st:p->steps){st.type=MACRO_STEP_ACT;st.u.act.cmd_id=ESP_CMD_MOTOR;st.u.act.duration_ms=100;}
  return p;
}
static void pending_completion_interrupt_test(const std::string& action) {
  macro_queues_init();commands_drained=false;uint32_t epoch=generation;
  macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=epoch;c.prog=moving_program();
  c.prog->count=1;c.prog->steps[0].u.act.duration_ms=0;CHECK(xQueueSend(macro_ctrl_queue,&c,0));
  int cycles=0;delay_hook=[&](){
    if(++cycles==1){
      macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));
      CHECK(p.state==MACRO_STATE_RUNNING);CHECK(commands.size()==1&&commands[0]==ESP_CMD_MOTOR);
      if(action=="stop"||action=="stop_resume"){
        esp_link_request_stop();if(action=="stop_resume")CHECK(esp_link_resume_commands(generation));
      } else if(action=="drain_race"){
        commands_drained=true;after_drain_check=[](){esp_link_request_stop();CHECK(esp_link_resume_commands(generation));};
      } else {
        macro_ctrl_t control{};control.generation=generation;
        if(action=="replace"){
          control.type=MACRO_CTRL_START;control.prog=moving_program();control.prog->id=91;allow_resume=false;
        } else control.type=MACRO_CTRL_ABORT;
        CHECK(xQueueSend(macro_ctrl_queue,&control,0));
      }
    } else if(cycles==2&&action=="replace"){
      CHECK(s_cur&&s_cur->id==91);CHECK(commands.size()==2&&commands[1]==ESP_CMD_STOP_ALL);
      CHECK(generation==epoch+1);allow_resume=true;
    } else throw TaskDone{};
  };
  try{macro_task(nullptr);}catch(TaskDone&){}
  macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));
  CHECK(commands[1]==ESP_CMD_STOP_ALL);CHECK(generation==epoch+1);
  if(action=="replace"){
    CHECK(p.state==MACRO_STATE_RUNNING);CHECK(p.id==91);CHECK(s_cur);
    CHECK(commands.size()==3&&commands[2]==ESP_CMD_MOTOR);
  } else {
    CHECK(p.state==MACRO_STATE_ABORTED);CHECK(p.id==90);CHECK(s_cur==nullptr);CHECK(commands.size()==2);
  }
}
static void safety_test(const std::string& name) {
  macro_queues_init();esp_net_queues_init();
  macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();
  xQueueSend(macro_ctrl_queue,&c,0);
  if(name=="abort_full"){
    for(int i=0;i<3;++i){c.prog=moving_program();xQueueSend(macro_ctrl_queue,&c,0);}
    handle_macro_abort();CHECK(inhibited);
  }
  int cycles=0;
  delay_hook=[&](){if(++cycles==1&&name=="stop_running")esp_link_request_stop();else throw TaskDone{};};
  try{macro_task(nullptr);}catch(TaskDone&){}
  CHECK(s_cur==nullptr);
  if(name=="stop_running"){macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));CHECK(p.state==MACRO_STATE_ABORTED);}
  else CHECK(std::find(commands.begin(),commands.end(),ESP_CMD_MOTOR)==commands.end());
}
static void stale_rx_test() {
  macro_queues_init();esp_net_queues_init();
  const char* line="{\"cmd\":\"LED_TOGGLE\"}";
  esp_net_rx_message_t message{strdup(line),generation,0,0,(uint32_t)strlen(line)};
  CHECK(message.line);CHECK(esp_net_rx_reserve(message.line_bytes));
  CHECK(xQueueSend(esp_net_rx_queue,&message,0));
  esp_link_request_stop();esp_link_resume_commands(generation);commands.clear();
  receive_budget=1;
  try{cmd_parser_task(nullptr);}catch(TaskDone&){}
  CHECK(commands.empty());
}
static void goto_parse_test() {
  macro_queues_init();esp_net_queues_init();
  cJSON* root=cJSON_Parse("{\"type\":\"macro\",\"id\":8,\"steps\":[{\"goto\":65536},{\"cmd\":\"LED_TOGGLE\"}]}");
  handle_macro(root,generation,0,0);cJSON_Delete(root);
  macro_ctrl_t c{};CHECK(xQueueReceive(macro_ctrl_queue,&c,0));
  CHECK(c.prog->steps[0].u.got.target<0||c.prog->steps[0].u.got.target>=c.prog->count);free(c.prog);
}
static void queue_oom_test(bool net) {
  fail_queue_create=true;bool failed=false;
  try {if(net)esp_net_queues_init();else macro_queues_init();}catch(const std::runtime_error&){failed=true;}
  CHECK(failed);
}
static void invalid_numbers_test(bool macro) {
  esp_net_queues_init();macro_queues_init();
  const char* command_cases[]={"{\"cmd\":\"MOTOR\",\"motor\":256,\"speed\":1}","{\"cmd\":\"MOTOR\",\"motor\":0,\"speed\":32768}","{\"cmd\":\"MOTOR_CLOSED\",\"motor\":0,\"speed_d100\":1e100}","{\"cmd\":\"PID\",\"kp\":1e100,\"ki\":1}"};
  const char* macro_cases[]={"{\"id\":65536,\"steps\":[{\"cmd\":\"LED_TOGGLE\"}]}","{\"id\":1,\"steps\":[{\"cmd\":\"LED_TOGGLE\",\"duration_ms\":-1}]}","{\"id\":1,\"steps\":[{\"wait\":{\"field\":\"dist_front_mm\",\"op\":\"<\",\"value\":10,\"timeout_ms\":-1}}]}","{\"id\":1,\"steps\":[{\"goto\":0.5}]}"};
  for(int i=0;i<4;++i){cJSON* root=cJSON_Parse(macro?macro_cases[i]:command_cases[i]);CHECK(root);if(macro)handle_macro(root,generation,0,0);else handle_command(root,generation,0,0);cJSON_Delete(root);CHECK(commands.empty());CHECK(macro_ctrl_queue->data.empty());}
}
static void run_parser_once(const char* json, uint32_t epoch) {
  // 直接注入 RX 队列也遵守生产预算契约；单客户端固定 slot=0、received_ms=0。
  esp_net_rx_message_t message{strdup(json),epoch,0,0,(uint32_t)strlen(json)};
  CHECK(message.line);CHECK(esp_net_rx_reserve(message.line_bytes));
  CHECK(xQueueSend(esp_net_rx_queue,&message,0));
  receive_budget=1;try{cmd_parser_task(nullptr);}catch(TaskDone&){}receive_budget=-1;
}
static void check_gen_ack(const char* name, int status) {
  char* out=nullptr;CHECK(xQueueReceive(esp_net_ack_queue,&out,0));
  cJSON* root=cJSON_Parse(out);free(out);CHECK(root);
  cJSON* ack=cJSON_GetObjectItem(root,"ack");
  cJSON* value=cJSON_GetObjectItem(root,"status");
  CHECK(cJSON_IsString(ack)&&strcmp(ack->valuestring,name)==0);
  CHECK(cJSON_IsNumber(value)&&value->valuedouble==status);cJSON_Delete(root);
}
static void gen_stale_test() {
  esp_net_queues_init();macro_queues_init();uint32_t old=generation;
  esp_link_request_stop();commands.clear();
  std::string json="{\"cmd\":\"MOTOR\",\"motor\":0,\"speed\":500,\"gen\":"+std::to_string(old)+"}";
  run_parser_once(json.c_str(),generation);
  CHECK(commands.empty());CHECK(inhibited);check_gen_ack("MOTOR",2);
  // A stale local envelope must not suppress the protocol's stale-gen ACK either.
  run_parser_once(json.c_str(),old);
  CHECK(commands.empty());CHECK(inhibited);check_gen_ack("MOTOR",2);
  json="{\"type\":\"macro\",\"id\":9,\"gen\":"+std::to_string(old)+",\"steps\":[{\"cmd\":\"LED_TOGGLE\"}]}";
  run_parser_once(json.c_str(),generation);
  CHECK(macro_ctrl_queue->data.empty());CHECK(commands.empty());CHECK(inhibited);check_gen_ack("macro",2);
}
static void gen_resume_test() {
  esp_net_queues_init();macro_queues_init();uint32_t old=generation;
  esp_link_request_stop();commands.clear();
  std::string json="{\"cmd\":\"MOTOR_CLOSED\",\"motor\":0,\"speed_d100\":4000,\"gen\":"+std::to_string(generation)+"}";
  run_parser_once(json.c_str(),generation);
  CHECK(!inhibited);CHECK(commands.size()==1&&commands[0]==ESP_CMD_MOTOR_CLOSED);check_gen_ack("MOTOR_CLOSED",0);
  // Explicit current gen also supersedes an older local envelope for START macros.
  json="{\"type\":\"macro\",\"id\":10,\"gen\":"+std::to_string(generation)+",\"steps\":[{\"cmd\":\"LED_TOGGLE\"}]}";
  run_parser_once(json.c_str(),old);
  macro_ctrl_t c{};CHECK(xQueueReceive(macro_ctrl_queue,&c,0));
  CHECK(c.type==MACRO_CTRL_START&&c.generation==generation);CHECK(c.prog&&c.prog->id==10);free(c.prog);check_gen_ack("macro",0);
}
static void gen_stop_all_test() {
  esp_net_queues_init();macro_queues_init();uint32_t old=generation;
  esp_link_request_stop();commands.clear();uint32_t stopped=generation;
  std::string json="{\"cmd\":\"STOP_ALL\",\"gen\":"+std::to_string(old)+"}";
  run_parser_once(json.c_str(),old);
  CHECK(inhibited);CHECK(generation==stopped+1);
  CHECK(commands.size()==1&&commands[0]==ESP_CMD_STOP_ALL);check_gen_ack("STOP_ALL",0);
}
static void race_test(bool macro) {
  esp_net_queues_init();macro_queues_init();
  before_enqueue=[](){esp_link_request_stop();esp_link_resume_commands(generation);};
  if(macro){macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();xQueueSend(macro_ctrl_queue,&c,0);delay_hook=[](){throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}CHECK(s_cur==nullptr);}
  else run_parser_once("{\"cmd\":\"LED_TOGGLE\"}",generation);
  CHECK(std::find(commands.begin(),commands.end(),macro?ESP_CMD_MOTOR:ESP_CMD_LED_TOGGLE)==commands.end());
}
static void stop_step_test(bool final_step=false) {
  macro_queues_init();macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();c.prog->steps[0].u.act.cmd_id=ESP_CMD_STOP_ALL;xQueueSend(macro_ctrl_queue,&c,0);
  if(final_step){c.prog->count=1;c.prog->steps[0].u.act.duration_ms=0;}commands_drained=false;
  int cycles=0;delay_hook=[&](){if(++cycles==3)throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}
  CHECK(s_cur==nullptr);CHECK(inhibited);CHECK(commands.size()==1&&commands[0]==ESP_CMD_STOP_ALL);
  macro_progress_t p{};CHECK(xQueuePeek(macro_progress_queue,&p,0));CHECK(p.state==MACRO_STATE_ABORTED);
}
static void pending_start_test() {
  macro_queues_init();esp_link_request_stop();allow_resume=false;macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();xQueueSend(macro_ctrl_queue,&c,0);
  int cycles=0;delay_hook=[&](){if(++cycles==1){CHECK(s_cur);CHECK(commands.size()==1);allow_resume=true;}else throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}
  CHECK(commands.size()==2&&commands[1]==ESP_CMD_MOTOR);
}
static void stale_start_test() {
  macro_queues_init();macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();xQueueSend(macro_ctrl_queue,&c,0);esp_link_request_stop();esp_link_resume_commands(generation);commands.clear();
  delay_hook=[](){throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}CHECK(s_cur==nullptr);CHECK(commands.empty());
}
static void disconnect_full_test() {
  macro_queues_init();for(int i=0;i<4;++i){macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();xQueueSend(macro_ctrl_queue,&c,0);}
  post_disconnect();CHECK(inhibited);delay_hook=[](){throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}CHECK(s_cur==nullptr);CHECK(commands.size()==1&&commands[0]==ESP_CMD_STOP_ALL);
}
static void strict_json_test() {
  esp_net_queues_init();macro_queues_init();run_parser_once("{\"cmd\":\"LED_TOGGLE\"} garbage",generation);CHECK(commands.empty());
  std::string too_many="{\"type\":\"macro\",\"id\":1,\"steps\":[";for(int i=0;i<65;++i){if(i)too_many+=",";too_many+="{\"cmd\":\"LED_TOGGLE\"}";}too_many+="]}";
  run_parser_once(too_many.c_str(),generation);CHECK(macro_ctrl_queue->data.empty());
}
static void wait_snapshot_test() {
  macro_queues_init();esp_net_queues_init();esp_link_sensor_queue=xQueueCreate(1,sizeof(esp_link_snapshot_t));
  esp_link_snapshot_t sensor{};sensor.sensor.dist_front_mm=100;
  // 同一 F407 时基下提供新鲜、有效的距离采样，适配 WAIT 的子系统新鲜度检查。
  sensor.sensor.sys_tick_ms=100;sensor.sensor.dist_timestamp_ms=100;sensor.sensor.dist_front_status=0;
  sensor.received_ms=xTaskGetTickCount();sensor.seq=1;xQueueOverwrite(esp_link_sensor_queue,&sensor);
  macro_ctrl_t c{};c.type=MACRO_CTRL_START;c.generation=generation;c.prog=moving_program();c.prog->steps[0].type=MACRO_STEP_WAIT;c.prog->steps[0].u.wait={MACRO_FIELD_DIST_FRONT_MM,MACRO_OP_LT,200,1000};c.prog->steps[1].u.act.duration_ms=0;xQueueSend(macro_ctrl_queue,&c,0);
  int cycles=0;delay_hook=[&](){if(++cycles==1){CHECK(s_pc==1);CHECK(xQueuePeek(esp_link_sensor_queue,&sensor,0));}else throw TaskDone{};};try{macro_task(nullptr);}catch(TaskDone&){}
  CHECK(s_cur==nullptr);CHECK(xQueuePeek(esp_link_sensor_queue,&sensor,0));
}
static void ack_oom_test() {
  esp_net_queues_init();cJSON_Hooks hooks={json_malloc,json_free};cJSON_InitHooks(&hooks);
  bool reached_success=false;
  for(int fault=0;fault<100;++fault){json_fail_at=fault;json_allocations=0;send_ack("LED_TOGGLE",0);bool hit=json_allocations>fault;char* out=nullptr;bool sent=xQueueReceive(esp_net_ack_queue,&out,0);if(out)json_free(out);CHECK(json_live==0);if(hit)CHECK(!sent);else{CHECK(sent);reached_success=true;break;}}
  CHECK(reached_success);cJSON_InitHooks(nullptr);
}
static void listener_failure_test(){esp_net_queues_init();host_next_socket=-1;bool exited=false;try{esp_net_task(nullptr);}catch(TaskDone&){exited=true;}CHECK(exited);CHECK(inhibited);CHECK(host_closed.empty());}
static void net_allocation_test(bool tx, bool duplicate) {
  esp_net_queues_init();macro_queues_init();sockaddr_in peer{};peer.sin_family=AF_INET;host_accept.push_back(peer);
  host_received[12]="{\"cmd\":\"LED_TOGGLE\"}\n";int cycles=0;
  select_hook=[&](fd_set* r,fd_set* w){FD_ZERO(r);if(w)FD_ZERO(w);if(++cycles==1)FD_SET(10,r);else if(cycles==2){fail_allocation_after=duplicate?1:0;if(tx){char* out=strdup("{\"n\":1}");xQueueSend(esp_net_tx_queue,&out,0);}else FD_SET(12,r);}else throw TaskDone{};};
  try{esp_net_task(nullptr);}catch(TaskDone&){}
  CHECK(inhibited);CHECK(esp_net_rx_queue->data.empty());CHECK(host_sent[12].empty());CHECK(std::find(host_closed.begin(),host_closed.end(),12)!=host_closed.end());
}
static void program_oom_test(){esp_net_queues_init();macro_queues_init();fail_allocation_after=0;run_parser_once("{\"type\":\"macro\",\"id\":1,\"steps\":[{\"cmd\":\"LED_TOGGLE\"}]}",generation);CHECK(macro_ctrl_queue->data.empty());char* out=nullptr;CHECK(xQueueReceive(esp_net_ack_queue,&out,0));CHECK(strstr(out,"\"status\":1"));free(out);}
static void rx_full_test(){
  esp_net_queues_init();macro_queues_init();
  const char* line="{\"cmd\":\"LED_TOGGLE\"}";
  for(int i=0;i<32;++i){
    esp_net_rx_message_t m{strdup(line),generation,0,0,(uint32_t)strlen(line)};
    CHECK(m.line);CHECK(esp_net_rx_reserve(m.line_bytes));
    CHECK(xQueueSend(esp_net_rx_queue,&m,0));
  }
  uint32_t old=generation;host_received[12]="{\"cmd\":\"STOP_ALL\"}\n";sockaddr_in peer{};host_accept.push_back(peer);int cycles=0;select_hook=[&](fd_set* r,fd_set* w){FD_ZERO(r);if(w)FD_ZERO(w);if(++cycles==1)FD_SET(10,r);else if(cycles==2)FD_SET(12,r);else throw TaskDone{};};try{esp_net_task(nullptr);}catch(TaskDone&){}CHECK(inhibited);CHECK(generation!=old);receive_budget=8;try{cmd_parser_task(nullptr);}catch(TaskDone&){}CHECK(commands.size()==1&&commands[0]==ESP_CMD_STOP_ALL);
}
int main(int argc, char **argv) {
  struct Case { const char *name; void (*run)(); };
  const Case cases[] = {
    {"newline", [](){newline_test();}},
    {"macro64", [](){receive_test(false);}},
    {"oversized", [](){receive_test(true);}},
    {"partial", [](){send_test(false);}},
    {"slow", [](){send_test(true);}},
    {"udp", [](){udp_test();}},
    {"video_enabled", [](){video_enabled_test();}},
    {"video_disconnect", [](){video_disconnect_test();}},
    {"video_overwrite", [](){video_ownership_test("overwrite");}},
    {"video_flush", [](){video_ownership_test("flush");}},
    {"video_nopeer", [](){video_ownership_test("nopeer");}},
    {"video_sendfail", [](){video_ownership_test("sendfail");}},
    {"video_inflight", [](){video_ownership_test("inflight");}},
    {"video_reject", [](){video_ownership_test("reject");}},
    {"goto_high", [](){macro_test("goto_high");}},
    {"goto_negative", [](){macro_test("goto_negative");}},
    {"progress", [](){macro_test("progress");}},
    {"completion_drain", [](){completion_drain_test("act");}},
    {"goto_high_drain", [](){completion_drain_test("goto_high");}},
    {"goto_negative_drain", [](){completion_drain_test("goto_negative");}},
    {"pending_completion_stop", [](){pending_completion_interrupt_test("stop");}},
    {"pending_completion_stop_resume", [](){pending_completion_interrupt_test("stop_resume");}},
    {"pending_completion_abort", [](){pending_completion_interrupt_test("abort");}},
    {"pending_completion_replace", [](){pending_completion_interrupt_test("replace");}},
    {"completion_drain_race", [](){pending_completion_interrupt_test("drain_race");}},
    {"snapshot", [](){snapshot_test();}},
    {"heartbeat_timeout_once", [](){heartbeat_timeout_test(false);}},
    {"heartbeat_latched", [](){heartbeat_timeout_test(true);}},
    {"activity_latch_reset", [](){activity_latch_reset_test();}},
    {"telemetry_no_client", [](){telemetry_client_gate_test(false);}},
    {"telemetry_reconnect", [](){telemetry_client_gate_test(true);}},
    {"missing_xbox_queue", [](){missing_xbox_queue_test();}},
    {"json_oom", [](){oom_test();}},
    {"stop_running", [](){safety_test("stop_running");}},
    {"abort_full", [](){safety_test("abort_full");}},
    {"stale_rx", [](){stale_rx_test();}},
    {"goto_parse", [](){goto_parse_test();}},
    {"net_queue_oom", [](){queue_oom_test(true);}},
    {"macro_queue_oom", [](){queue_oom_test(false);}},
    {"invalid_numbers", [](){invalid_numbers_test(false);}},
    {"invalid_macro_numbers", [](){invalid_numbers_test(true);}},
    {"command_race", [](){race_test(false);}},
    {"macro_race", [](){race_test(true);}},
    {"stop_step", [](){stop_step_test();}},
    {"final_stop_step", [](){stop_step_test(true);}},
    {"pending_start", [](){pending_start_test();}},
    {"stale_start", [](){stale_start_test();}},
    {"disconnect_full", [](){disconnect_full_test();}},
    {"strict_json", [](){strict_json_test();}},
    {"wait_snapshot", [](){wait_snapshot_test();}},
    {"ack_oom", [](){ack_oom_test();}},
    {"listener_failure", [](){listener_failure_test();}},
    {"rx_oom", [](){net_allocation_test(false,false);}},
    {"rx_copy_oom", [](){net_allocation_test(false,true);}},
    {"tx_oom", [](){net_allocation_test(true,false);}},
    {"program_oom", [](){program_oom_test();}},
    {"rx_full", [](){rx_full_test();}},
    {"gen_stale", [](){gen_stale_test();}},
    {"gen_resume", [](){gen_resume_test();}},
    {"gen_stop_all", [](){gen_stop_all_test();}},
  };
  if (argc != 2) return 2;
  if (std::string(argv[1]) == "--list") {
    for (const auto &test : cases) std::puts(test.name);
    return 0;
  }
  try {
    for (const auto &test : cases) if (std::string(argv[1]) == test.name) {
      test.run(); std::printf("PASS %s\n", test.name); return 0;
    }
    throw std::runtime_error("unknown test");
  } catch (const std::exception &e) {
    std::fprintf(stderr, "FAIL %s: %s\n", argv[1], e.what()); return 1;
  }
}
