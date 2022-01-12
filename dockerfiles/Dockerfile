FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04
LABEL org.opencontainers.image.title="ROFT Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing ROFT experiments"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/hsp-iit/roft/master/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Nicola A. Piga <nicola.piga@iit.it>"

# Use /bin/bash instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Set the locale
RUN apt update && \
    apt install -y -qq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt update && \
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake cmake-curses-gui curl emacs-nox git glmark2 gnupg2 htop iputils-ping lsb-release mesa-utils nano psmisc sudo unzip vim wget && \
    rm -rf /var/lib/apt/lists/*

# Install GitHub cli
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt update && \
    apt install --no-install-recommends -y -qq gh && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt update && \
    apt install --no-install-recommends -y -qq libassimp-dev libglfw3-dev libglew-dev libglm-dev libeigen3-dev libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# Build bayes-filters-lib
RUN git clone https://github.com/xenvre/bayes-filters-lib && \
    cd bayes-filters-lib && git checkout of-aided-tracking && mkdir build && cd build && \
    cmake .. && \
    make install

# Build superimpose-mesh-lib
RUN git clone https://github.com/robotology/ycm && \
    cd ycm && mkdir build && cd build && \
    cmake .. && \
    make install
RUN git clone https://github.com/xenvre/superimpose-mesh-lib && \
    cd superimpose-mesh-lib && git checkout of-aided-tracking && mkdir build && cd build && \
    cmake .. && \
    make install

# Build RobotsIO
RUN git clone https://github.com/xenvre/robots-io && \
    cd robots-io && mkdir build && cd build && \
    cmake .. && \
    make install

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user

# Configure emacs
RUN echo "(setq-default indent-tabs-mode nil)" >> /home/user/.emacs.el && \
    echo "(setq-default tab-width 4)" >> /home/user/.emacs.el && \
    echo "(setq make-backup-files nil)" >> /home/user/.emacs.el && \
    echo "(setq auto-save-default nil)" >> /home/user/.emacs.el && \
    echo "(setq c-default-style \"linux\"" >> /home/user/.emacs.el && \
    echo "      c-basic-offset 4)" >> /home/user/.emacs.el && \
    echo "(global-subword-mode 1)" >> /home/user/.emacs.el && \
    echo "(add-hook 'before-save-hook 'delete-trailing-whitespace)" >> /home/user/.emacs.el && \
    echo "(custom-set-variables '(custom-enabled-themes '(tango-dark)))" >> /home/user/.emacs.el && \
    echo "(custom-set-faces)" >> /home/user/.emacs.elx

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
