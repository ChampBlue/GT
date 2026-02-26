

#add_library : 타겟을 하나 만든다 -> add_library(name TYPE sources ..)는 name이라는 타겟을 만들고 그 타겟이 어떤 산출물을 만들지(type)와 어떤 소스들로 만들지(sources)를 정의함.
#라이브러리 파일을 만든다가 1차목적, 그 다음 그 라이브러리를 다른 타겟이 target_link_libraries()로 가져다 쓰는 구조임.
#STATIC : add_library(led STATIC drivers/led/led.c) -> led.c를 컴파일해서 오브젝트를 만들고, 그걸 묶어서 정적 라이브러리 libled.a를 만든다
#           최종 exe 링크할 때 libled.a의 필요한 코드가 exe안으로 복사되어 들어감.

#SHARED : add_library(led SHARED ...) -> 공유라이브러리를 만듦(.so) 피코에선 보통 안끔

#INTERFACE : add_library(board_cfg INTERFACE) : 아무것도 컴파일 안함. 산출물도 없음. 대신 이 타겟에 붙인 설정( include dir / define / options 같은)만 이 타겟을 링크하는 다른 타겟에 전파함.
add_library(board_cfg INTERFACE)
target_include_directories(board_cfg INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/board/include
)



#add_library(led STATIC...)로 만들면 libled.a(정적 라이브러리)가 생기고, 누구든지 target_link_libraries(어떤 타겟 led)로 링크하면 사용 가능함.
add_library(led STATIC
    drivers/led/led.c
)


# drivers/led 폴더를 include 검색 경로로 추가한다는 뜻
# led 타겟에 포함된 모든 소스파일에 대해 컴파일할 떄 -Idrivers/led가 붙음 -> #include "led.h"같은게 찾아짐.
# 그리고 PUBLIC이므로 led를 링크하는 타겟(예: board, m10)에도 그 include경로가 전파되어, 그 타겟에서도 #include "led.h"가 가능해짐.
# PUBLIC VS INTERFACE
#  led.h를 외부 타겟(board, m10)이 include 해야한다 -> 전파가 필요 -> PUBLIC 또는 INTERFACE후보
#  그런데 led타겟 자기 자신(led.c)도 #include "led.h"를 해야함 -> 자기 컴파일에도 include 경로 필요 -> PUBLIC 또는 PRIVATE 후보
# INTERFACE는 나 자신(led 타겟) 컴파일에 적용안되고 나를 링크하는 쪽에만 전파함.
target_include_directories(led PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/drivers/led
)


# led를 컴파일 할 떄 필요하는 것들을 연결해줌. 
# board_cfg : board/include 경로 (= board_config.h)찾기
# hardware_gpio: gpio_init, gpio_put 같은 함수 심볼 제공
#pico_stdlib: sleep_ms같은 기본기능/ 타이밍계열 제공
target_link_libraries(led PUBLIC
    board_cfg
    pico_stdlib
    hardware_gpio
)

##############################################################
add_library(board_cfg INTERFACE)
target_include_directories(board_cfg INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/board/include
)

add_library(board STATIC
    board/src/board_init.c
)
target_link_libraries(board PUBLIC
    board_cfg
    pico_stdlib
    hardware_clocks
    hardware_timer
    led
    motor
)

add_library(led_header INTERFACE)
target_include_directories(led_header INTERFACE ${CMAKE_CURRENT_LIST_DIR}/drivers/led)

add_library(led STATIC drivers/led/led.c)
target_link_libraries(led PUBLIC
    led_header
    pico_stdlib
)