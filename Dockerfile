FROM ros:melodic-robot-bionic

ENV CODE_MOUNT /workspaces
ENV ROS_WS /ros
ENV PYTEST_ADDOPTS "--color=yes"
SHELL [ "bash", "-c"]
WORKDIR /root

# Install pip 
RUN apt update \
	&& apt install -y \
		libffi-dev \
		libssl-dev \
		python-pip \
	&& pip install --no-cache-dir --upgrade pip \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Install Python testing packages
RUN pip install \
		pytest \
		pytest-cov \
		coveralls

# Install OS packages
RUN apt update \
	&& apt install -y \
		bash-completion \
		htop \
		vim \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Workaround for faiing dep install on osrf/ros image https://stackoverflow.com/a/48569233
# RUN python -m easy_install --upgrade pyOpenSSL

RUN pip install \
	Adafruit-GPIO \
	Adafruit-MCP3008 \
	spidev \
	pyserial

# RUN mkdir -p ${CODE_MOUNT} && \
# 	mkdir -p ${CODE_MOUNT}/b2-base && \
# 	mkdir -p ${ROS_WS}/src && \
# 	ln -s ${CODE_MOUNT}/b2-base/b2_base ${ROS_WS}/src/b2_base && \
# 	ln -s ${CODE_MOUNT}/roboclaw_driver ${ROS_WS}/src/roboclaw_driver

RUN mkdir -p ${CODE_MOUNT} \
&& cd ${CODE_MOUNT} \
&& git clone https://github.com/sheaffej/roboclaw_driver.git

COPY ./b2_base ${CODE_MOUNT}/

RUN mkdir -p ${ROS_WS}/src && \
	ln -s ${CODE_MOUNT}/b2_base ${ROS_WS}/src/b2_base && \
	ln -s ${CODE_MOUNT}/roboclaw_driver ${ROS_WS}/src/roboclaw_driver

# RUN echo "source ${CODE_MOUNT}/b2-base/mybashrc" >> /root/.bashrc && \
# 	echo "source /root/.bashrc" >> /root/.bash_profile

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
	apt-get update && \
	cd $ROS_WS && catkin_make && \
	rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*