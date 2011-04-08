Readme file for FRIComponent.

The FRIComponent provides an interface between OROCOS and the
KUKA.fastresearchinterface.

Under example you can find an OROCOS example configuration file for
joint-space position control.

Steps:

- compile/link the FRIComponent
- adjust the example/test_LWR_naxes.xml file: set correct paths
- adjust the example/cpf/FRIServer.cpf file: set correct port
- adjust the example/cpf/nAxesGeneratorPos.cpf file: choose maximum velocity
  and acceleration values

- use the KUKA KCP to set the LWR into FRI monitor mode
- deploy the test_LWR_naxes.xml file using the OCL deployer
- check the connection between OROCOS and FRI: within the deployer type 'cd
  FRIServer' and then 'ls' to get an overview of the FRIServer. You should now be
able to read the actual robot joint positions et cetera.

the connection between the OROCOS program and FRI should be working! Do not proceed if it doesn't work!

- start the nAxesGeneratorPos component to initialize the trajectory generator:
  within the deployer type 'cd ..' (if necessary). Then type 'nAxesGeneratorPos.start()'
- use the KUKA KCP to set the LWR into FRI command mode
- execute a movetTo command:
  'nAxesGeneratorPos.moveTo(array(axis1,axis2, ...,
  axis7), minimaltime)', where axisi corresponds to the desired position in radians of axis i and minimaltime the minimal time taken to execute the motion.









