sudo apt-get -y install libnss3-tools
curl -L "https://dl.filippo.io/mkcert/latest?for=linux/amd64" -o mkcert
chmod +x mkcert
sudo mv mkcert /usr/local/bin/mkcert
mkcert --install
cp $(mkcert-v1.4.4-linux-arm64 -CAROOT)/* .
mkcert localhost
