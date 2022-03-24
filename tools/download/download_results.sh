SERVER=https://dataverse.iit.it
PERSISTENT_ID=doi:10.48557/9FJW42
TOKEN=

echo "Downloading results..."
curl -L -H X-Dataverse-key:$TOKEN $SERVER/api/access/dataset/:persistentId/?persistentId=$PERSISTENT_ID -o results.zip
unzip -qq results.zip
rm results.zip
rm MANIFEST.TXT
rm README.txt
