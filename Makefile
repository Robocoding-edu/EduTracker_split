
update: restore pull setup reset

setup:
	chmod +x ./server/start.sh
	chmod +x ./data/start.sh

reset:
	rm -rf ./data/build
	rm -rf ./data/install


restore:
	git restore ./server/start.sh
	git restore ./data/start.sh

pull:
	git pull

start-server:
	cd ./server && ./start.sh

start-rpi:
	docker compose up -d


arduino-build:
	cd ./arduino && pio run --target upload --upload-port /dev/sensors/arduino
