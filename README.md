LiteHouse là hệ thống nhà thông minh dựa trên STM32, cho phép người dùng:

Điều khiển thiết bị gia dụng (đèn, cửa, quạt, rèm cửa) qua ứng dụng Bluetooth trên điện thoại (MIT App Inventor).

Giám sát an toàn: phát hiện khí gas, cháy và tự động bật còi báo động, mở cửa, gửi cảnh báo đến điện thoại.

Dự án thực hiện bởi nhóm 3 thành viên

Phần cứng sử dụng

STM32 (STM32F103C8T6 “Blue Pill”)

Module Bluetooth HC-05

Cảm biến khí gas MQ-2

Relay module (điều khiển thiết bị AC)

Servo motor (mô phỏng cửa / rèm)

Buzzer + LED cảnh báo

Nguồn 5V / PCB custom

Phần mềm & Công nghệ

Ngôn ngữ: Embedded C (STM32 HAL)

IDE: STM32CubeIDE / Keil C

App điều khiển: MIT App Inventor (Android)

Kết nối: UART (Bluetooth HC-05)

Tính năng chính

Điều khiển thiết bị:

Bật/tắt đèn, quạt qua app điện thoại.

Đóng/mở cửa bằng servo.

Bluetooth Control:

App MIT App Inventor gửi lệnh qua UART → STM32 xử lý → điều khiển relay/servo.

Hệ thống an toàn:

MQ-2 phát hiện khí gas.

Tự động kích hoạt còi báo động (buzzer).

Gửi cảnh báo về điện thoại.

Mở cửa thoát hiểm bằng servo.

Ứng dụng MIT App Inventor

Giao diện gồm các nút: Đèn ON/OFF, Quạt ON/OFF, Cửa OPEN/CLOSE.

Có thêm nhãn hiển thị cảnh báo (GAS ALERT, FIRE ALERT).

Khi phát hiện khí gas, app sẽ nhận được thông báo ngay lập tức.

Cách triển khai

Nạp firmware vào STM32.

Kết nối Bluetooth HC-05 với app điện thoại.

Chạy app → gửi lệnh điều khiển thiết bị. ( tải app bằng cách tải files .aia ở trên branch lên MIT APP INVENTOR xong tải files apk về máy sử dụng)

Thử nghiệm tính năng gas alarm bằng MQ-2.

Demo: https://drive.google.com/drive/folders/1IyHeAX215S69uxc0pfSo29VWZflstxH-?usp=sharing

Tác giả và đóng góp: 

Nguyễn Hoàng Minh Quốc

Nguyễn Kim Thành

Mạc Quang Khải
