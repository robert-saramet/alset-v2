
from flask import Flask, json, render_template, request, jsonify
from herepy.models import RoutingResponse
from herepy.routing_api import RoutingApi
import requests
import urllib.parse
from geopy.geocoders import Nominatim

class GPS:
	def __init__(self):
		self.api_key = 'OGlw5e3t9mUKmredPM_dWAHcIng_8IWvwd9M_PnLaoY'
		self.api = RoutingApi(self.api_key, 3000)
		self.current_location = None
	def update_current_location(self):
		pass
	def get_route(self, destination):
		return self.api.car_route(self.current_location, destination)

app = Flask(__name__)


def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()

@app.route('/')
def map_func():
	return render_template('index.html')

@app.route('/test', methods=['GET', 'POST'])
def test_func():
	# init the APIs
	gps = GPS()
	geolocator = Nominatim(user_agent='good_solid_user_agent')

	address = request.get_json()["destination"]
	location = geolocator.geocode(address)
	if location == None:
		return jsonify({ 'status': 0 })

	gps.current_location = (request.get_json()['current_lat'], request.get_json()['current_long'])

	route = gps.get_route((location.latitude, location.longitude))
	
	route_coords = []
	for value in route.as_dict()['response']['route'][0]['leg'][0]['maneuver']:
		route_coords.append((value['position']['latitude'], value['position']['longitude']))

	end_coords_json = route.as_dict()['response']['route'][0]['leg'][0]['end']['mappedPosition']
	end_coords = (end_coords_json['latitude'], end_coords_json['latitude'])

	print("\n\n", route_coords, "\n\n")

	return jsonify({ 'status': 1 })

if __name__ == '__main__':
	app.run()

