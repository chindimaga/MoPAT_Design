from flask import render_template, request
import json
from WebUI import app


@app.route("/", methods=["GET", "POST", 'OPTIONS'])
def index_page():
	return render_template('index.html')


@app.route('/hook', methods=["GET", "POST", 'OPTIONS'])
def process():
    xstr = request.values['x']
	# ystr = request.values['y']
    return xstr
