#!/usr/bin/guile -s
!#
;; script to extract the firmware from a windows driver file
;; (c) uuh 2007, herewith put into public domain
;;
;; inspection showed that the windows driver file 
;; SYSFILE contains blobs of firmware data mixed with the URB request data
;;
;;  <1-octet transfer length> for TransferBufferLength
;;  <2-octet lsb dest value>      for Value slot of request
;;  <1-octet index>           for Index slot of request
;; For the .fw files that we use we need the 4th octet thrown out, 
;; and leave the length in place (since INDEX is always 0x00)

;; the file that contains the firmware 
(define *driverfile* "R5U870FLx86.sys")
;; where to put the result
(define *destfile* "r5u870_183b.fw")
;; start address of firmware data in *DRIVERFILE* 
(define *start* 46224)
;; length of data
(define *length* 16981)

(define *expected-md5sum* #f)

;;; no user-serviceable parts below


;; make sure we get sane byte behaviour
(setlocale LC_ALL "C")

(use-modules (ice-9 format))

(define (read-N-bytes count)
  "Read N bytes from CURRENT-INPUT-PORT, failing hard if not enough are there."
  (with-output-to-string 
    (lambda ()
      (let loop ((remaining count))
	(if (> remaining 0)
	    (begin (write-char (read-char))
		   (loop (- remaining 1))))))))

(define (write-N-bytes count data)
  "Write N bytes from DATA to CURRENT-OUTPUT-PORT, failing hard if not enough are there."
  (with-input-from-string data
    (lambda ()
      (let loop ((remaining count))
	(if (> remaining 0)
	    (begin (write-char (read-char))
		   (loop (- remaining 1))))))))

(define (driversegment->fw string)
  "Read chunks of <len-byte> <low address byte> <high address byte> <index byte> <LEN bytes data chunk> from FILE and return a string where the <index-byte> has been dropped."
  (with-output-to-string 
    (lambda ()
      (with-input-from-string string
	(lambda ()
	  (let loop ()
	    (if (eof-object? (peek-char))
		(format (current-error-port) "done~%")
		(begin
		  (let* ((lenbyte (read-char))
			 (lenint (char->integer lenbyte))
			 (addrbyte1 (read-char))
			 (addrbyte2 (read-char))
			 (ignoredbyte (read-char))
			 (data (read-N-bytes lenint)))
		    (format (current-error-port)
			    "~d -> ~x; " lenint
			    (logior (char->integer addrbyte1)
				    (* 256 (char->integer addrbyte2))))
		    (format #t "~c~c~c" lenbyte addrbyte1 addrbyte2)
		    (write-N-bytes lenint data)
		    (loop))))))))))

(define (extract-slice file start length)
  "Return bytes START through START+LENGTH from FILE as a string"
  (with-input-from-file file
    (lambda ()
      (seek (current-input-port) start SEEK_SET)
      (read-N-bytes length))))

;; main:
(format (current-error-port)
	"Decoding firmware from ~A into ~A:~%" *driverfile* *destfile*)
(with-output-to-file *destfile*
  (lambda ()
    (let ((extracted-data (driversegment->fw
			   (extract-slice *driverfile* *start* *length*))))
      (format (current-error-port)
	      "Result has length ~d~%" (string-length extracted-data) )
      (write-N-bytes (string-length extracted-data) extracted-data))))