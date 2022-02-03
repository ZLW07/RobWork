
input="$1/version"
MAJOR=0
MINOR=0
PATCH=0
DEBREV=1

I=0
while IFS= read -r line
do
    I=$(expr $I + 1 )
    extension="${line##*.}"
    base="${line%.*}"
    case $base in
        major ) MAJOR=$extension ;;
        minor ) MINOR=$extension ;;
        patch ) PATCH=$extension ;;
        debrev ) DEBREV=$extension ;;
    esac
done < "$input"

if [ ! $I -gt 3 ] ; then
    echo "0.0.0"
    exit 1
fi

if [[ $3 == "-p1" ]] ; then 
    PATCH=$((PATCH + 1))
fi


if [[ $2 == "-n" ]] ; then
    echo "$MAJOR.$MINOR.$PATCH"
else
    echo "-DVERSION=$MAJOR.$MINOR.$PATCH"
fi
exit 0