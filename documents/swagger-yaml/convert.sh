#/bin/bash

## https://swagger2markup.github.io/swagger2markup/1.3.1/#_docker_image
## https://swagger2markup.github.io/swagger2markup/1.3.1/#_swagger2markup_properties

if [[ `docker images | grep swagger2markup` == "" ]]
then
    docker pull swagger2markup/swagger2markup
fi

for f in cp*.yaml
do
    out=`basename -s .yaml $f`
    if [ "$f" -ot "$out" ]
    then
	docker run --rm -v $(pwd):/opt swagger2markup/swagger2markup convert -i /opt/$f -f /opt/$out -c /opt/config.properties
    else
	echo "[[[[ skipping $f because it hasn't changed ]]]]"
    fi
done
