from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# 조이스틱 함수

def calculate_motor_power(x, y):
    steering_sensitivity = 0.6
    x = x * steering_sensitivity

    left_motor = y + x
    right_motor = y - x

    max_val = max(abs(left_motor), abs(right_motor))
    if max_val > 1.0:
        left_motor /= max_val
        right_motor /= max_val

    return int(left_motor * 100), int(right_motor * 100)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    x = data.get('x', 0)
    y = data.get('y', 0)

    left_pwm, right_pwm = calculate_motor_power(x, y)

    print(f"🎮 조이스틱: (X={x:.2f}, Y={y:.2f}) -> 🚗 모터 출력: [L: {left_pwm}, R: {right_pwm}]")

    return jsonify({'status': 'success', 'L': left_pwm, 'R': right_pwm})

if __name__ == '__main__':
    app.config['TEMPLATES_AUTO_RELOAD'] = True
    app.run(host='0.0.0.0', port=5000, debug=True)