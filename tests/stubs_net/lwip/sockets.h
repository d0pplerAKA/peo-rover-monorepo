#pragma once
#include <winsock2.h>
using socklen_t = int;
#undef FD_ISSET
inline int host_fd_isset(int fd,const fd_set* set) {for(unsigned i=0;i<set->fd_count;++i)if(set->fd_array[i]==(SOCKET)fd)return 1;return 0;}
#define FD_ISSET(fd,s) host_fd_isset(fd,s)
#include <cerrno>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include "freertos/task.h"
inline int host_next_socket=10;
inline std::function<void(fd_set*,fd_set*)> select_hook;
inline std::map<int,std::string> host_received,host_sent;
inline std::map<int,size_t> host_send_limit;
inline std::vector<int> host_closed;
inline std::vector<std::pair<sockaddr_in,std::string>> host_udp;
inline int host_sendto_errno = 0;
inline std::function<void()> host_sendto_hook;
inline std::vector<sockaddr_in> host_accept;
inline int host_socket(int,int,int) { return host_next_socket++; }
inline int host_bind(int,const sockaddr*,int) {return 0;}
inline int host_listen(int,int) {return 0;}
inline int host_setsockopt(int,int,int,const void*,int) {return 0;}
inline int host_close(int fd) {host_closed.push_back(fd);return 0;}
inline int host_accept_fn(int,sockaddr* addr,int* len) {if(host_accept.empty()){errno=EAGAIN;return -1;} if(addr&&len)memcpy(addr,&host_accept.front(),sizeof(sockaddr_in));host_accept.erase(host_accept.begin());return host_next_socket++;}
inline int host_recv(int fd,void* p,size_t n,int) {auto &s=host_received[fd]; if(s.empty()) return 0; n=std::min(n,s.size());memcpy(p,s.data(),n);s.erase(0,n);return (int)n;}
inline int host_send(int fd,const void* p,size_t n,int) {if(host_send_limit.count(fd))n=std::min(n,host_send_limit[fd]);if(!n){errno=EAGAIN;return -1;}host_sent[fd].append((const char*)p,n);return (int)n;}
inline int host_sendto(int,const void* p,size_t n,int,const sockaddr* a,int) {if(host_sendto_hook){auto hook=host_sendto_hook;hook();}if(host_sendto_errno){errno=host_sendto_errno;return -1;}if(!a){errno=EDESTADDRREQ;return -1;}host_udp.push_back({*(const sockaddr_in*)a,std::string((const char*)p,n)});return (int)n;}
inline int host_select(int,fd_set* r,fd_set* w,fd_set*,timeval*) {if(select_hook)select_hook(r,w);return 1;}
inline int host_fcntl(int,int,...) {return 0;}
#define socket host_socket
#define bind host_bind
#define listen host_listen
#define setsockopt host_setsockopt
#define close host_close
#define accept host_accept_fn
#define recv host_recv
#define send host_send
#define sendto host_sendto
#define select host_select
#define fcntl host_fcntl
#define htons(v) ((uint16_t)((((uint16_t)(v))>>8)|(((uint16_t)(v))<<8)))
#define htonl(v) ((uint32_t)__builtin_bswap32((uint32_t)(v)))
#define F_GETFL 3
#define F_SETFL 4
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x4000
#endif
