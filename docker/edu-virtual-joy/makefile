image_name = eduart-virtual-joy:0.1.0

download-sources:
	git clone --branch feature/ros2 https://github.com/EduArt-Robotik/edu_virtual_joy.git

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: download-sources build-docker
	$(info Creating EduArt's robot library Docker image "$(image_name)".)

clean:
	rm -rf edu_robot
