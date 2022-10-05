create table 회원(
	회원아이디 varchar(30) not null,
    비밀번호 varchar(30) not null,
    성명 varchar(30) not null,
    비행기번호 int,
    좌석번호 int,
    primary key(회원아이디)
);
CREATE TABLE 신용카드(
  신용카드번호 CHAR(16) NOT NULL,
  유효기간 CHAR(4) NOT NULL,
  PRIMARY KEY(신용카드번호)
);
CREATE TABLE 회원_신용카드(
  회원아이디 VARCHAR(30) NOT NULL,
  신용카드번호 CHAR(16) NOT NULL,
  primary key(회원아이디, 신용카드번호),
  FOREIGN KEY(회원아이디) REFERENCES 회원(회원아이디) ON DELETE CASCADE ON UPDATE CASCADE,
  FOREIGN KEY(신용카드번호) REFERENCES 신용카드(신용카드번호) ON DELETE CASCADE ON UPDATE CASCADE
);
CREATE TABLE 비행기(
  비행기번호 INT NOT NULL,
  출발날짜 DATE NOT NULL,
  출발시간 TIME NOT NULL,
  PRIMARY KEY(비행기번호)
);
CREATE TABLE 좌석(
  비행기번호 int not null,
  좌석번호 int not null,
  등급정보 VARCHAR(30) not null,
  회원아이디 varchar(30),
  PRIMARY KEY(비행기번호, 좌석번호),
  FOREIGN KEY(비행기번호) REFERENCES 비행기(비행기번호) on delete cascade on update cascade,
  foreign key(회원아이디) references 회원(회원아이디) on delete cascade on update cascade
);

alter table 회원
add foreign key(비행기번호, 좌석번호) references 좌석(비행기번호, 좌석번호) on delete cascade on update cascade;