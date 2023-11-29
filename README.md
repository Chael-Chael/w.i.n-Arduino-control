# 简介
这是HUST校机器人比赛w.i.n团队电控组的Github仓库，储存一些控制有关的内容和资源。

# 电机驱动

## A4988控制的步进电机驱动
### 教程详解
>http://www.taichi-maker.com/homepage/reference-index/motor-reference-index/arduino-a4988-nema-stepper-motor/#circuit2

### 参数
>There is a test point for VREF on the Pololu but it is missing on the Stepstick. Since it is just the wiper of the pot you can measure it there and it is easier as it is a bigger target. I hold the positive meter probe on the shaft of a metal screwdriver so I can see the value while I am turning the pot. Put the negative probe on a ground pin.

To calculate the current, A = VREF / (8 * RS). For a standard stepstick, RS is the rating of the Sense Resistor = 0.2ohm. So A = VREF / 1.6

To calculate the VREF for a target current, VREF = A * 8 * RS , or A * 1.6 . So, if you wanted 0.8A, VREF = 0.8 * 1.6 = 1.28V

### 使用的42步进电机
>https://item.taobao.com/item.htm?id=578042430410&price=11-11.4&sourceType=item&sourceType=item&suid=51faf97e-fa0b-4939-bc2e-632cda6284de&shareUniqueId=24389834830&ut_sk=1.WkkJr1TxX0IDAJacdkLjIQj6_21646297_1700997448893.TaoPassword-QQ.1&un=9dbbe77b736875bdaf0acc16981d2ad5&share_crt_v=1&un_site=0&spm=a2159r.13376460.0.0&sp_abtk=gray_1_code_simpleAndroid2&tbSocialPopKey=shareItem&sp_tk=ZjdRM1dWU09rUTI%3D&cpp=1&shareurl=true&short_name=h.5PrV46K&bxsign=scdZ-guty1LeKHqPamr0rZ7ehz_8TpoaXsbJwEp7rorYIgIJUIeDaBG8EX6N_OAXcqq0-UWjYC1A1w32MJPYm_3ecgnY_G70cVK6kPrleFtXa3VAHZLPS8azsnxSLKrU7_YqvL-Zr2Ah6uS8PVCsquVjA&tk=f7Q3WVSOkQ2&app=chrome

## 舵机
### 使用的舵机
>[淘宝]https://m.tb.cn/h.5lUqSTN?tk=SaFuWVxb0OU CZ3457 「MG996R MG946R MG995 金属标准舵机180度360度数码机器人20KG舵机」
