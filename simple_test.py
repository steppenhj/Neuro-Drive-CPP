from flask import Flask

app = Flask(__name__)

@app.route('/')
def index():
    return "<h1>CONNECTION SUCCESS!</h1><p>Neuro-Drive Network is OK.</p>"

if __name__ == '__main__':
    # 0.0.0.0은 모든 IP에서의 접속을 허용한다는 뜻입니다.
    print("Starting Simple Server on port 5000...")
    app.run(host='0.0.0.0', port=5000, debug=True)