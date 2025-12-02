from manim import *

class HallSensorMultiScan(Scene):
    def construct(self):
        # --- 1. 설정 및 그리드 생성 ---
        rows = 5
        cols = 5
        sensor_size = 0.8
        spacing = 0.2
        
        grid = VGroup()
        sensors = [[None for _ in range(cols)] for _ in range(rows)]
        
        for r in range(rows):
            for c in range(cols):
                sensor = Square(side_length=sensor_size)
                sensor.set_stroke(color=WHITE, width=2)
                sensor.set_fill(color=GRAY, opacity=0.5)
                x_pos = (c - cols/2 + 0.5) * (sensor_size + spacing)
                y_pos = (rows/2 - r - 0.5) * (sensor_size + spacing)
                sensor.move_to([x_pos, y_pos, 0])
                sensors[r][c] = sensor
                grid.add(sensor)
        
        row_labels = VGroup()
        for r in range(rows):
            label = Text(f"R{r}", font_size=20).next_to(sensors[r][0], LEFT)
            row_labels.add(label)
        col_labels = VGroup()
        for c in range(cols):
            label = Text(f"C{c}", font_size=20).next_to(sensors[0][c], UP)
            col_labels.add(label)

        title = Text("Multi detection Hall Sensor Matrix", font_size=36)
        title.to_edge(UP)

        self.play(Write(title))
        self.play(Create(grid), Write(row_labels), Write(col_labels))
        
        # --- 2. 자석 배치 ---
        magnet_positions = [(1, 1), (3, 2), (3, 4)]
        magnets_group = VGroup()
        
        for r_pos, c_pos in magnet_positions:
            magnet = Circle(radius=0.3, color=RED, fill_opacity=0.8)
            magnet_text = Text("N", color=WHITE, font_size=20).move_to(magnet)
            single_magnet_grp = VGroup(magnet, magnet_text)
            
            # 해당 센서 위치로 이동
            single_magnet_grp.move_to(sensors[r_pos][c_pos].get_center())
            magnets_group.add(single_magnet_grp)
            
        info_text = Text(f"Placing {len(magnet_positions)} Magnets...", font_size=24, color=YELLOW).next_to(grid, DOWN)
        self.play(Write(info_text))
        # 자석 생성 에니메이션
        self.play(FadeIn(magnets_group, shift=UP*0.5, lag_ratio=0.1), run_time=1.5)
        self.wait(1)

        # --- 3. 인식 시각화 ---
        scan_text = Text("Scanning...", font_size=20).next_to(grid, DOWN)
        self.play(Transform(info_text, scan_text))
        
        detected_sensors = []

        # 행을 하나씩 훑으면서 검사
        for r in range(rows):
            # 3-1. 현재 행 활성화 - 노란색 테두리
            row_indicator = SurroundingRectangle(VGroup(*sensors[r]), color=YELLOW, buff=0.1)
            row_status = Text(f"Activate R{r} -> Read C0-C4", font_size=20, color=YELLOW).next_to(row_indicator, RIGHT)
            self.play(Create(row_indicator), Write(row_status), run_time=0.3)
            
            # 3-2. 해당 행의 열 검사
            sensors_on_this_row_to_light_up = []
            for c in range(cols):
                if (r, c) in magnet_positions:
                    sensors_on_this_row_to_light_up.append(sensors[r][c])
                    detected_sensors.append((r,c))

            if len(sensors_on_this_row_to_light_up) > 0:
                animations = [sensor.animate.set_fill(GREEN, opacity=1) for sensor in sensors_on_this_row_to_light_up]
                animations.append(Flash(sensors_on_this_row_to_light_up[0], color=GREEN, flash_radius=0.5, run_time=0.5))
                
                self.play(*animations, run_time=0.5)
                self.wait(1.0)
            else:
                # 자석이 없는 행은 빠르게 지나감
                self.wait(0.5)
            
            # 표식 제거
            self.play(FadeOut(row_indicator), FadeOut(row_status), run_time=0.2)

        # --- 4. 마무리 ---
        result_str = ", ".join([str(pos) for pos in detected_sensors])
        final_text = Text(f"Detected Positions: {result_str}", font_size=20, color=GREEN)
        final_text.next_to(grid, DOWN)
        self.play(Transform(info_text, final_text))
        self.wait(3)
