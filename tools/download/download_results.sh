wget -qO- --save-cookies cookies.txt --keep-session-cookies --no-check-certificate "https://dataverse.iit.it/privateurl.xhtml?token=16a1d3f6-4861-4e14-b339-602d7bc8326d" &> /dev/null
wget --load-cookies cookies.txt "https://dataverse.iit.it/api/access/datafile/1768?gbrecs=true" -O results.tar.gz
rm cookies.txt
tar xzf results.tar.gz
rm results.tar.gz
