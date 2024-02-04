FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    wget git curl unzip \
    vim sudo zsh

# install oh my zsh & change theme to af-magic
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && \
    ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc &&\
    chsh -s /bin/zsh

RUN apt-get install -y \
    nodejs npm

RUN npm cache clean --force && \
    npm config set registry https://registry.npmmirror.com

RUN npm install hexo hexo-cli -g