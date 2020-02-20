# Find-Path-
Find Path using A* algorithm
      1. Vấn đề:
Trên bảng đồ sẽ xuất hiện thêm một số điểm khác nhau (được gọi là điểm đón). Tìm đường đi xuất phát từ S đến G  sao cho lần lượt đi qua các điểm này và phải sắp xếp thứ tự ưu tiên đi qua chúng sao cho đoạn đường đi là ngắn nhất.
      2 Cài đặt:
- Đầu tiên đọc các tham số cần thiết từ file input.txt bằng hàm readfile(). 
- Tiếp theo, ta sẽ đi tìm thứ tự ưu tiên đi qua các điểm đón nào trước bằng hàm FindPriority(). Quy ước chi phí được tính là quãng đường đi giữa 2 điểm, nếu đi dọc hay ngang thì chi phí mỗi bước là 1, nếu đi chéo thì chi phí là 1,5. Quá trình tìm này dựa vào ý tưởng thuật toán AStar với hàm ước lượng heristic f(x). Trước hết, ta sẽ tìm một điểm gần G nhất trong số các điểm đón rồi đưa điểm đó vào tập đóng stack, cứ tiếp tục tìm trong tập các điểm đón còn lại gần nhất với điểm trên top của stack, cho đến khi đi về điểm S. 
- Sau khi đã có thứ tự ưu tiên đi qua mỗi điểm thì ta sẽ tìm đường đi chi tiết từ S đi qua các điểm đó và đến G sao cho tránh các vật thể đã cho bằng hàm FindPath(). Cuối cùng là xuất kết quả ra file output.txt và biểu diễn đồ họa đường đi.
