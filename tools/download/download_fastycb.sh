mkdir -p dataset/
[ ! -d dataset/fast-ycb ] && git clone https://github.com/hsp-iit/fast-ycb dataset/fast-ycb
cd dataset/fast-ycb
bash tools/download/download_dataset.sh
