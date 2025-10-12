# Build docker image
```
docker build --platform linux/arm64 -t gazebo-web-arm64 .
```

# Run docker
```
docker run --shm-size=2g --memory=8g --memory-swap=12g --rm -it \
  -p 8080:8080 \
  -p 7681:7681 \
  -p 6080:6080 \
  -p 5900:5900 \
  --platform linux/arm64 \
  gazebo-web-arm64
```