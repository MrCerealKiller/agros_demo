#!/bin/bash

set -e

HEAD=$(tput bold; tput setaf 5)
SEC=$(tput bold; tput setaf 6)
SUCC=$(tput bold; tput setaf 2)
WARN=$(tput bold; tput setaf 3)
ERR=$(tput bold; tput setaf 1)
RES=$(tput sgr0)

if [[ "$(whoami)" == "root" ]]; then
  echo "${ERR}Running as root may cause unwanted permissions${RES}"
  echo "Exiting..."
  exit -1
fi

echo
echo "${HEAD}    AgROS Demo Setup Script${RES}"
echo

sudo -v

# Get location of the repository and cd there ---------------------------------
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
echo "Script found in ${DIR}"
echo
cd ${DIR}

# Udev Rules ------------------------------------------------------------------
echo "${SEC}Symlinking the udev rules to global configuration...${RES}"
if [[ ! -f "/etc/udev/rules.d/14-agros.rules" ]]; then
  echo "Linking..."
  RULES=${DIR}/config/14-agros.rules
  sudo ln -sf ${RULES} /etc/udev/rules.d/

  echo "Reloading udev..."
  sudo udevadm control --reload
  sudo udevadm trigger
  echo "Done"
  echo
else
  echo "File already exists. Skipping..."
  echo
fi

# Tmuxinator Project ----------------------------------------------------------
echo "${SEC}Symlinking the tmuxinator project to user configuration...${RES}"
if [[ ! -f "${HOME}/.tmuxinator/agros.yaml" ]]; then
  echo "Linking..."
  TMUXINATOR=${DIR}/config/tmuxinator.yml
  sudo ln -sf ${TMUXINATOR} ${HOME}/.tmuxinator/agros.yml
  echo "Done"
  echo
else
  echo "File already exists. Skipping..."
  echo
fi

# UCenter ---------------------------------------------------------------------
echo "${SEC}Getting the uCenter installer...${RES}"
echo "${WARN}Note: the isntaller requires Wine and must be installed${RES}"
if [[ ! -d "${DIR}/ignore/ucenter" ]]; then
  # Link to download the uCenter installer
  echo "Getting zip from U-Blox..."
  UCENTER=https://www.u-blox.com/sites/default/files/u-centersetup_v19.04.zip
  wget --directory-prefix=ignore/ucenter/ $UCENTER

  # Unzip the package
  echo "Unzipping..."
  ZIPFILE="$(ls ignore/ucenter | grep "u-center")"
  unzip ignore/ucenter/${ZIPFILE} -d ignore/ucenter
  echo "Cleaning up..."
  rm ignore/ucenter/${ZIPFILE}
  echo "Done"
  echo
else
  echo "ignore/ucenter directory already exists. Skipping..."
  echo
fi

# Symlink /dev/gps to COM 14 --------------------------------------------------
echo "${SEC}Linking /dev/gps to wine COM port...${RES}"
echo "Checking to see if wine is installed..."
if [[ -x  "$(command -v wine)" ]]; then
  echo "Wine is installed"

  # Loop over input until nothing or a valid number is received
  while true; do
    echo -n "What COM port would you like linked to the /dev/gps?"
    read -r -p " (default: 99; 's' to skip) " port
    if [[ ${port} =~ ^[0-9]+$ ]]; then
      echo "Linking to com${port}..."
      ln -s /dev/gps ~/.wine/dosdevices/com${port}
      echo "Done"
      echo
      break
    elif [[ $port == "s" ]]; then
      echo "Skipping..."
      echo "Done"
      echo
      break
    elif [[ $port == "" ]]; then
      echo "Linking to com99..."
      ln -s /dev/gps ~/.wine/dosdevices/com99
      echo "Done"
      echo
      break
    else
      echo "${ERR}Invalid input. Please try again.${RES}"
    fi
  done

else
  echo "Wine is not installed"
  echo "Please install wine and link /dev/gps to a wine COM port manually"
  echo "Skipping..."
  echo
fi

# Add Agros Path to zshrc as AGROS_DIR ----------------------------------------
# Loop over input until nothing or a valid number is received
while true; do
  echo "Add agrosrc as another source in your zshrc?"
  echo "It is recommended for things like the tmuxinator project to work"
  echo -n "but you can configure it yourself, manually."
  read -r -p " (yN) " addPath
  if [[ ${addPath} == "y" ]]; then
    echo "Appending source command to your zshrc..."
      echo "# Link source to agros shell utils" >> ${HOME}/.zshrc
      echo "source ${DIR}/agrosrc" >> ${HOME}/.zshrc
      echo "Done"
      echo
      break
    elif [[ $addPath == "n" || $addPath == "N"  || $addPath == "" ]]; then
      echo "Skipping..."
      echo "Done"
      echo
      break
    else
      echo "${ERR}Invalid input. Please try again.${RES}"
    fi
  done

# Misc Configs ----------------------------------------------------------------
echo "${SEC}Adding user to dialout groups...${RES}"
sudo usermod -aG dialout $(whoami)
echo "Done"
echo

echo "${SUCC}Setup complete.${RES}"
echo
