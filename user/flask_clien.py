import cv2
import requests
import numpy as np

def main():
    # Flask 서버에서 제공하는 video_feed URL (서버 주소와 포트를 맞춰주세요)
    url = 'http://192.168.14.80:5000/video_feed'
 
    # streaming 모드로 HTTP 요청
    stream = requests.get(url, stream=True)
    bytes_data = b''

    try:
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            # JPEG 시작(FFD8)과 끝(FFD9) 마커를 찾아서 프레임 추출
            start = bytes_data.find(b'\xff\xd8')
            end = bytes_data.find(b'\xff\xd9')
            if start != -1 and end != -1:
                jpg = bytes_data[start:end+2]  # JPEG 이미지 데이터 추출
                bytes_data = bytes_data[end+2:]  # 사용한 데이터를 버림
                # numpy 배열로 변환 후, JPEG 디코딩하여 이미지 생성
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("Video Feed", frame)
                    # ESC 키를 누르면 종료 (키코드 27)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()