
update: docker-down restore pull setup reset

docker-down:
	docker compose down

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
	./server/start.sh

start-rpi:
	docker compose up -d
