sudo apt-get -y install libnss3-tools
curl -L "https://dl.filippo.io/mkcert/latest?for=linux/arm64" -o mkcert
chmod +x mkcert
sudo mv mkcert /usr/local/bin/mkcert
mkcert --install
mkcert localhost
mv localhost.pem localhost.original.pem
cat localhost.original.pem $(mkcert -CAROOT)/rootCA.pem > localhost.pem
