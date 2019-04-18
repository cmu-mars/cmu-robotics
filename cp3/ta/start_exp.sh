#/bin/bash
. ~/.bashrc
#sudo chown -R mars:mars /home/mars/logs /home/mars/.ros/log
echo "rainbow.consider_cost=$CONSIDER_RECONFIGURATION_COST" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow.properties
echo "rainbow.consider_cost=$CONSIDER_RECONFIGURATION_COST" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow-cp3_ta.properties


echo "rainbow.max_adaptations=$MAX_RAINBOW_ADAPTATIONS" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow.properties
echo "rainbow.max_reconfs=$MAX_RAINBOW_ADAPTATIONS" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow-cp3_ta.properties

echo "rainbow.balanced_utility=$BALANCED_UTILITY" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow.properties
echo "rainbow.balanced_utility=$BALANCED_UTILITY" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow-cp3_ta.properties

echo "rainbow.fix_path=$FIX_PATH" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow.properties
echo "rainbow.fix_path=$FIX_PATH" >> /home/mars/das/rainbow-brass/targets/brass-p2-cp3/rainbow-cp3_ta.properties

python3 -m swagger_server $@
