#/bin/bash -x

RR1=7b2713e450425ead250d804a00012c599ad5da61
RR2=6ebbf11ce21e61d7505115deb272f8502f876eae
RR2IW=09447e13a8a966f38437b478448674fcf1e25fb5

SHA=""

case $1 in
    rr1|RR1)
	SHA=$RR1
    ;;
    rr2|RR2)
	SHA=$RR2
    ;;
    rr2iw|RR2IW|rr2-iw)
	SHA=$RR2IW
    ;;
    *)
	echo "unknown or missing first arugment"
	exit 1
esac

for f in *.yaml
do
    echo "-----------------------------------------------------------------------" >> changes-since-$1.txt
    echo $f >> changes-since-$1.txt
    echo "-----------------------------------------------------------------------" >> changes-since-$1.txt
    git diff -w $SHA $f >> changes-since-$1.txt
done
