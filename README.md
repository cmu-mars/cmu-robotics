cmu-robotics
============

Vagrant installation can be found at: https://www.vagrantup.com/docs/installation/

In order to run the simulation, once you've installed vagrant, etc.:

```
vagrant up
vagrant ssh
./run-cp1.sh
```

and wait until you see `odom received!` in one of the info messages in the
output. Then, from either the host machine or inside the vagrant guest,
you can access the REST communications API with standard HTTP requests that
meet the API from the wiki on port 5000. For example, at a new terminal,

```
% curl -X POST -d "" localhost:5000/phase1/power/start_challenge_problem
starting challenge problem
%
```

will start the simulation. You can see the debugging output in the terminal
that's running `vagrant ssh` into the guest machine, or issue futher HTTP
requests per the REST API.

You will also see logging information about the state of the robot. When you see:

```
... Goal reached
```

on the terminal, that means the robot has successfully completed.

(Note that this logging information will go away in future releases, probably. 
This was written when we thought we had to write the code to validated the 
intents, and it is what we were going to use. There will be another path through
the API to get that kind of information.)
