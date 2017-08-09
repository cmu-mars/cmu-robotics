#/bin/bash

for f in cp*.yaml
do
    docker run --rm -v $(pwd):/opt swagger2markup/swagger2markup convert -i /opt/$f -f /opt/`basename -s .yaml $f` -c /opt/config.properties
done
