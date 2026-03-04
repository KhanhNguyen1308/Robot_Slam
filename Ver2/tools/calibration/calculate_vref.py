#!/usr/bin/env python3
"""
Hướng dẫn đo và điều chỉnh VREF cho A4988
"""

def calculate_vref():
    """Tính VREF cần thiết cho motor"""
    print("=" * 70)
    print("HƯỚNG DẪN TÍNH VÀ ĐIỀU CHỈNH VREF CHO A4988")
    print("=" * 70)
    print()
    print("VREF quyết định dòng điện (current limit) cho motor.")
    print("Nếu quá thấp: motor yếu, không quay được")
    print("Nếu quá cao: motor nóng, A4988 có thể hỏng")
    print()
    print("=" * 70)
    print()
    
    # Get motor rated current
    print("BƯỚC 1: Xác định rated current của motor")
    print("-" * 70)
    print("Nhìn vào nhãn trên motor NEMA 17, tìm thông số:")
    print("  - Rated Current hoặc Current/Phase")
    print("  - Thường là: 1.0A, 1.2A, 1.5A, 1.7A, hoặc 1.8A")
    print()
    
    while True:
        try:
            rated_current = input("Nhập rated current của motor (A) [mặc định 1.5]: ").strip()
            if rated_current == "":
                rated_current = 1.5
            else:
                rated_current = float(rated_current)
            
            if 0.5 <= rated_current <= 2.5:
                break
            else:
                print("⚠ Giá trị không hợp lệ. NEMA 17 thường 0.5A - 2.5A")
        except ValueError:
            print("⚠ Vui lòng nhập số hợp lệ")
    
    print(f"\n✓ Motor rated current: {rated_current}A")
    
    # Calculate safe current limit (70% of rated)
    print()
    print("BƯỚC 2: Tính current limit an toàn")
    print("-" * 70)
    print("Để motor không quá nóng, nên đặt current limit = 70% rated current")
    print()
    
    current_limit = rated_current * 0.7
    print(f"Current limit khuyến nghị: {rated_current} × 0.7 = {current_limit:.3f}A")
    
    # Calculate VREF
    print()
    print("BƯỚC 3: Tính VREF cần đặt")
    print("-" * 70)
    print("Công thức cho A4988:")
    print("  VREF = Current_Limit / 2.5")
    print()
    
    vref = current_limit / 2.5
    print(f"VREF cần đặt: {current_limit:.3f} / 2.5 = {vref:.3f}V")
    print()
    print("=" * 70)
    print(f"⭐ KẾT QUẢ: ĐẶT VREF = {vref:.3f}V (khoảng {vref*1000:.0f}mV)")
    print("=" * 70)
    
    # Measurement instructions
    print()
    print("BƯỚC 4: ĐO VÀ ĐIỀU CHỈNH VREF")
    print("-" * 70)
    print("Dụng cụ cần thiết:")
    print("  - Đồng hồ vạn năng (multimeter)")
    print("  - Tuốc nơ vít nhỏ (để vặn potentiometer)")
    print()
    print("Cách đo VREF:")
    print("  1. BẬT nguồn 12V cho A4988 (motor có thể chưa cắm)")
    print("  2. Chỉnh đồng hồ về chế độ đo DC Voltage (VDC)")
    print("  3. Chân ĐEN (COM) của đồng hồ → Chạm vào GND của A4988")
    print("  4. Chân ĐỎ (V) của đồng hồ → Chạm vào đỉnh con đinh vít trên A4988")
    print("     (Đinh vít copper/kim loại màu đồng ngay trên chip A4988)")
    print()
    print("⚠ QUAN TRỌNG:")
    print("  - Cẩn thận không chạm vào các chân khác")
    print("  - Không làm ngắn mạch GND với VMOT")
    print()
    print("Điều chỉnh:")
    print("  - VẶN THUẬN chiều kim đồng hồ (phải) → TĂNG VREF")
    print("  - VẶN NGƯỢC chiều kim đồng hồ (trái) → GIẢM VREF")
    print("  - Vặn từ từ, từng chút một")
    print(f"  - Mục tiêu: đạt {vref:.3f}V (± 0.05V là OK)")
    print()
    
    # Additional current settings
    print("CÁC MỨC CURRENT KHÁC:")
    print("-" * 70)
    print(f"{'Mode':<20} {'Current (A)':<15} {'VREF (V)':<15} {'Ghi chú'}")
    print("-" * 70)
    
    options = [
        ("Safe (70%)", rated_current * 0.7, "Khuyến nghị - motor mát"),
        ("Moderate (80%)", rated_current * 0.8, "Cân bằng sức mạnh/nhiệt"),
        ("Full (100%)", rated_current * 1.0, "Tối đa - motor có thể nóng"),
    ]
    
    for mode, current, note in options:
        vref_calc = current / 2.5
        print(f"{mode:<20} {current:<15.3f} {vref_calc:<15.3f} {note}")
    
    print("-" * 70)
    print()
    
    # Test procedure
    print("BƯỚC 5: KIỂM TRA SAU KHI ĐIỀU CHỈNH")
    print("-" * 70)
    print("Sau khi đặt VREF:")
    print("  1. Tắt nguồn")
    print("  2. Cắm motor vào A4988 (nếu chưa cắm)")
    print("  3. Bật lại nguồn")
    print("  4. Chạy: sudo python3 test_motor_simple.py")
    print("  5. Quan sát:")
    print("     ✓ Motor quay mượt → VREF đúng!")
    print("     ✗ Motor yếu, không quay → TĂNG VREF lên")
    print("     ✗ Motor nóng quá → GIẢM VREF xuống")
    print("     ✗ Motor rung/kêu nhưng không quay → Kiểm tra MS pins!")
    print()
    
    # Common motor specs
    print()
    print("BẢNG THAM KHẢO: CÁC LOẠI NEMA 17 PHỔ BIẾN")
    print("=" * 70)
    print(f"{'Model':<25} {'Current':<12} {'VREF@70%':<12} {'Holding Torque'}")
    print("-" * 70)
    
    motors = [
        ("17HS4401", "1.7A", "0.48V", "40 N·cm"),
        ("17HS8401", "1.8A", "0.50V", "48 N·cm"),
        ("17HS4023", "1.0A", "0.28V", "23 N·cm"),
        ("17HS3001", "1.3A", "0.36V", "28 N·cm"),
        ("17HS2408", "1.2A", "0.34V", "23 N·cm"),
        ("42BYGH", "1.5A", "0.42V", "40 N·cm"),
    ]
    
    for model, current, vref, torque in motors:
        print(f"{model:<25} {current:<12} {vref:<12} {torque}")
    
    print("=" * 70)
    print()
    
    # Save to file option
    print("Lưu kết quả này?")
    save = input("Nhấn Y để lưu vào file vref_settings.txt [Y/n]: ").strip().lower()
    
    if save != 'n':
        with open("vref_settings.txt", "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("CẤU HÌNH VREF CHO A4988 + NEMA 17\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Motor rated current: {rated_current}A\n")
            f.write(f"Current limit (70%): {current_limit:.3f}A\n")
            f.write(f"VREF cần đặt: {vref:.3f}V ({vref*1000:.0f}mV)\n")
            f.write("\n")
            f.write("Các mức current khác:\n")
            for mode, current, note in options:
                vref_calc = current / 2.5
                f.write(f"  {mode}: {current:.3f}A → VREF = {vref_calc:.3f}V\n")
            
        print(f"✓ Đã lưu vào vref_settings.txt")
    
    print()
    print("=" * 70)
    print("HOÀN TẤT!")
    print("=" * 70)
    print()
    print("BƯỚC TIẾP THEO:")
    print("  1. Đo và điều chỉnh VREF theo hướng dẫn trên")
    print("  2. Kiểm tra MS pins (không để floating!)")
    print("  3. Chạy: sudo python3 test_motor_simple.py")
    print("  4. Đọc A4988_DIAGNOSTIC.md để biết thêm chi tiết")
    print()


if __name__ == "__main__":
    try:
        calculate_vref()
    except KeyboardInterrupt:
        print("\n\n⚠ Đã hủy bởi người dùng")
    except Exception as e:
        print(f"\n✗ Lỗi: {e}")
        import traceback
        traceback.print_exc()
