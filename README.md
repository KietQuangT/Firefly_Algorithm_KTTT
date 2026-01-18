Đây là code về thuật toán firefly và áp dụng nó lên bài toán UAV-RIS-trajectory
Chi tiết về thuật toán và thuật toán khi được áp dụng lên bài toán UAV-RIS-trajectory nằm trong file src
Để chạy hệ thống UAV-RIS-trajectory cần những điều kiện sau:
  - Mã này được triển khai bằng YALMIP và MOSEK. Vui lòng đảm bảo rằng bạn đã cài đặt đúng các hộp công cụ này trước khi chạy mã.
  - Các tập lệnh chạy bắt đầu bằng "main_" để plot.
  - Muốn chạy hệ thống khi áp dụng firefly thì chỉ cần thêm "_firefly" vào tên 2 hàm sau "opt_traj" và "opt_traj_noRIS"
