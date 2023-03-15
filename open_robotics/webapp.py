from flask import Flask, jsonify
from flask_cors import CORS
import numpy as np
from robot_analysis import RobotAnalysis

app = Flask(__name__)
CORS(app)

@app.route('/api/v1/data', methods=['GET'])
def get_data():
    ra = RobotAnalysis()
    ra.initDefaultData()
    print(ra.data.shape)
    return jsonify(ra.data.tolist())

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
