#!/usr/bin/perl -n

# export splash.png to splash.c in GIMP
# perl makesplash.pl <splash.c >splash.h

/^\s+"(.*)",?/ or next;
$all .= length > 1 ? eval "\"$_\"" : $_ foreach split /(?:\\[0-7]{3}|\\.|.)\K/, $1;

END {

	foreach (unpack ('S*', $all), undef) {
		$_ = (($_ & 0xe000) >> 8) | (($_ & 0x0700) >> 6) | (($_ & 0x0018) >> 3) if defined $_; # RGB565 >> RGB332

		if ($white and ($white == 255 or $_ != 0xff)) {
			printf '%d, ', $white;
			$white = 0;
		}
		if ($black and ($black == 255 or $_ != 0x00)) {
			printf '%d, ', $black;
			$black = 0;
		}

		if (defined $_ and $white == 0 and $black == 0) {
			printf '0x%x, ', $_;
		}


		if ($_ == 0xff) {
			$white++;
		} elsif (defined $_ and $_ == 0x00) {
			$black++;
		}
	}
}
