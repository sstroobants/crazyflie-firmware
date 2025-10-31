for i in {0..3}
do
  echo "Flashing device $i"
  CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E70$i" make cload
  # Add your flashing command here
done