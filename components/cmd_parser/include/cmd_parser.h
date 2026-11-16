#ifndef __CMD_PARSER_H__
#define __CMD_PARSER_H__

/**
 * @file    cmd_parser.h
 * @brief   PEO Rover - 上位机命令解析（TCP JSON -> esp_link 命令帧）
 *
 * cmd_parser_task（优先级 4）：阻塞等 esp_net_rx_queue 的 JSON 行，
 * 解析 {"cmd": 名字, 参数...} -> 编码 cmd_id+param -> esp_link 命令队列；
 * 成功提交后回发 ACK {"ack":名字,"status":0} 到 esp_net_ack_queue，由网络任务优先广播。
 *
 * 宏（{"type":"macro"}）已支持 act/wait/goto：解析校验后编译为 START 消息移交宏引擎。
 * STOP_ALL/PING/macro_abort 豁免本地陈旧代次过滤；macro_abort 直接锁 STOP。
 */

void cmd_parser_task(void *arg);

#endif /* __CMD_PARSER_H__ */
