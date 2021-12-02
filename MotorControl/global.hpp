/**
 * @file global.hpp
 * @brief 
 * @author htkang (htkang616@outlook.com)
 * @version 1.0
 * @date 2021-04-30
 * 
 * @copyright Copyright (c) 2021  苏州钧舵机器人有限公司
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-04-30 <td>1.0     <td>htkang     <td>内容
 * </table>
 */
 
#ifndef __GLOBAL_HPP
#define __GLOBAL_HPP

#include <stdint.h>

class Motor;
class OpticalEncoder;
class Controller;


extern Controller g_ctrl;
extern Motor g_motor;
extern OpticalEncoder g_optical_encoder;

#endif
