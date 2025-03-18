/**
 * @file platform_detection.hpp
 * @brief Platform detection.
 *
 * This header file contains macros to detect the platform on which the code is being compiled.
 * It supports detection for ESP32 and various STM32 platforms.
 *
 * @author Laurent Lardinois
 * @date January 2025
 */

//-----------------------------------------------------------------------------//
// C++ Publish/Subscribe Pattern - Spare time development for fun              //
// (c) 2025 Laurent Lardinois https://be.linkedin.com/in/laurentlardinois      //
//                                                                             //
// https://github.com/type-one/PublishSubscribeESP32                           //
//                                                                             //
// MIT License                                                                 //
//                                                                             //
// This software is provided 'as-is', without any express or implied           //
// warranty.In no event will the authors be held liable for any damages        //
// arising from the use of this software.                                      //
//                                                                             //
// Permission is granted to anyone to use this software for any purpose,       //
// including commercial applications, and to alter itand redistribute it       //
// freely, subject to the following restrictions :                             //
//                                                                             //
// 1. The origin of this software must not be misrepresented; you must not     //
// claim that you wrote the original software.If you use this software         //
// in a product, an acknowledgment in the product documentation would be       //
// appreciated but is not required.                                            //
// 2. Altered source versions must be plainly marked as such, and must not be  //
// misrepresented as being the original software.                              //
// 3. This notice may not be removed or altered from any source distribution.  //
//-----------------------------------------------------------------------------//

#pragma once

#if !defined(PLATFORM_DETECTION_HPP_)
#define PLATFORM_DETECTION_HPP_

#if defined(ESP_PLATFORM)
#define FREERTOS_PLATFORM
#elif defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F303xC) || defined(STM32F401xC)                     \
    || defined(STM32F401xE) || defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Vx)                    \
    || defined(STM32F446xx) || defined(STM32F756xx) || defined(STM32F765xx) || defined(STM32H743xx)                    \
    || defined(STM32H723xx) || defined(STM32H7) || defined(STM32H5)
#define STM32_PLATFORM
#endif

#if defined(STM32_PLATFORM)
#define FREERTOS_PLATFORM
#endif

#endif //  PLATFORM_DETECTION_HPP_
