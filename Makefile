# Common tasks, one name each. `make help` lists them. Nothing here deletes anything.
# Every recipe prints the command it runs; run from the repo root (make does that for you).

IMAGE   ?= motion-planner:build
PROFILE ?= src/description/mobile_manipulator_description/config/profiles/ur10e_telescopic.yaml
DOCKER  ?= docker
TAG     ?= motion-planner

.DEFAULT_GOAL := help
.PHONY: help doctor doctor-gpu check host-tests py-tests lint robot-tools robot-assets \
        images image-deps image-build image-sim-gazebo image-gpu image-full

help: ## Show this list
	@echo "Targets (override with VAR=value, e.g. make check IMAGE=motion-planner:dev):"
	@grep -E '^[a-zA-Z_-]+:.*## ' Makefile | sed -e 's/:.*## /\t/' | awk -F'\t' '{printf "  %-18s %s\n", $$1, $$2}'
	@echo ""
	@echo "Variables: IMAGE=$(IMAGE)  TAG=$(TAG)  PROFILE=$(PROFILE)"

doctor: ## Check this machine (disk, Docker, GPU, network, ROS env); add ARGS=--repo or --quiet
	docker/tools/doctor.sh $(ARGS)

doctor-gpu: ## Doctor plus a real docker --gpus smoke test (needs the gpu image)
	docker/tools/doctor.sh --gpu $(ARGS)

check: ## Colcon build and test, Python tests, repo scans in the IMAGE container (long)
	docker/run_all_checks.sh --image $(IMAGE) $(ARGS)

host-tests: ## ROS-free C++ core tests with g++ on the host
	docker/tools/run_host_core_tests.sh $(ARGS)

py-tests: ## Python test suites on the host (needs pytest, numpy, pyyaml, matplotlib, xacro)
	docker/tools/run_host_python_tests.sh

lint: ## Public-readiness scan, parameter docs check, doc citation check
	python3 docker/tools/check_public_readiness.py
	python3 docker/tools/check_params.py
	python3 docker/tools/check_doc_citations.py

images: image-build image-sim-gazebo image-gpu image-full ## Build the build, sim-gazebo, gpu and full images (very long)

image-deps: ## Build the deps image (TAG:deps)
	$(DOCKER) build -f docker/Dockerfile --target deps -t $(TAG):deps .

image-build: ## Build the build image (TAG:build)
	$(DOCKER) build -f docker/Dockerfile --target build -t $(TAG):build .

image-sim-gazebo: ## Build the Gazebo simulation image (TAG:sim-gazebo)
	$(DOCKER) build -f docker/Dockerfile --target sim-gazebo -t $(TAG):sim-gazebo .

image-gpu: ## Build the GPU image with cuRobo (TAG:gpu)
	$(DOCKER) build -f docker/Dockerfile --target gpu -t $(TAG):gpu .

image-full: ## Build the full image, gpu plus VAMP (TAG:full)
	$(DOCKER) build -f docker/Dockerfile --target full -t $(TAG):full .

robot-tools: ## Build the FOAM and Cricket model-generation images (clones into ~/.cache)
	docker/tools/build_robot_tools.sh

robot-assets: ## Generate URDF, SRDF, spheres and more from PROFILE (writes under build/)
	docker/tools/generate_robot_assets.sh $(PROFILE) $(ARGS)
