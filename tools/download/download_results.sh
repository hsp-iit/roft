SERVER=https://dataverse.iit.it
TOKEN=

curl -L -H X-Dataverse-key:$TOKEN $SERVER/api/access/datafile/1768 -o results.tar.gz
tar xzf results.tar.gz
rm results.tar.gz
