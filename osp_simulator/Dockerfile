FROM ubuntu:22.04

WORKDIR /sim

ADD osp /sim

RUN apt update && apt install -y tmux

RUN chmod +x /sim/cosim-demo-app/bin/cosim-demo-app &&\
    chmod +x /sim/cosim-cli/bin/cosim &&\
    chmod +x /sim/osp_config_gen_linux/osp_config_gen &&\
    chmod +x /sim/osp_config_gen_linux/run_config_gen.sh &&\
    echo 'export PATH=$PATH:/sim/cosim-cli/bin' >> ~/.bashrc &&\
    echo 'export PATH=$PATH:/sim/cosim-demo-app/bin/' >> ~/.bashrc &&\
    echo 'export PATH=$PATH:/sim/osp_config_gen_linux' >> ~/.bashrc


EXPOSE 8000

CMD "/bin/bash"
