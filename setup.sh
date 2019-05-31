echo "Installing uCenter GPS utility. It requires Wine to run."
UCENTER=https://www.u-blox.com/sites/default/files/u-centersetup_v19.04.zip
wget --directory-prefix=ignore/ $UCENTER
ZIPFILE=$(ls ignore | grep "u-center")
unzip ignore/$ZIPFILE -d ignore/
rm ignore/$ZIPFILE
